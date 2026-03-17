#pragma once
/*
 * kws.h
 * ─────────────────────────────────────────────────────────────
 * KWS (Keyword Spotting) — Binary voice activity / command detector
 *
 * WHAT IT DOES:
 *   Runs continuously on a shared 16000-sample ring buffer.
 *   Classifies audio as "valid command" vs "irrelevant/noise".
 *   Returns true when a valid command is detected above threshold.
 *   On true, the ring buffer contents are passed directly to SLU
 *   — no re-capture, same 1s window.
 *
 * WHAT IT DOES NOT DO:
 *   There is no wakeword. KWS is binary — it simply decides whether
 *   the current audio window contains a valid command utterance.
 *   SLU then classifies what that command means.
 *
 * RING BUFFER OWNERSHIP:
 *   The ring buffer is declared in MedReminder.ino as a global.
 *   kws.h reads from it and writes to it via I2S.
 *   slu.h reads from it (read-only) when KWS triggers.
 *   initKWS() owns I2S driver installation — must be called before
 *   anything else reads from I2S.
 *
 * RING BUFFER MECHANICS:
 *   - 16000 int16 samples = exactly 1 second at 16kHz
 *   - New samples written at ring_head, wrapping around
 *   - Every 160 samples (10ms), KWS runs inference on a linearised
 *     copy of the ring buffer contents
 *   - On KWS trigger, ring buffer is frozen and passed to SLU
 *
 * ── HOW TO PLUG IN YOUR MODEL ────────────────────────────────
 * 1. Export Edge Impulse project as Arduino library, install it
 * 2. Uncomment the #include and update the library name
 * 3. Update MODEL PARAMS to match your project
 * 4. Uncomment the TODO inference block in runKWSInference()
 * 5. Replace the kws_flat[] placeholder with EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
 *
 * Public API:
 *   void  initKWS()
 *   bool  runKWSInference()
 *     — reads new I2S samples into ring buffer
 *     — every KWS_STRIDE_SAMPLES, runs inference
 *     — returns true if valid command detected
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <driver/i2s.h>

// ── Model header placeholder ──────────────────────────────────
// TODO: uncomment when Edge Impulse library is installed
// #include <KWS_inferencing.h>

// ============================================================
// MODEL PARAMS — edit this section to match your model
// ============================================================

// ── Binary classification label ───────────────────────────────
// The label string in your Edge Impulse project that means
// "this audio contains a valid command"
// The other label (noise/irrelevant) is everything else
#define KWS_POSITIVE_LABEL    "command"    // TODO: confirm label name

// ── Confidence threshold ──────────────────────────────────────
constexpr float KWS_CONFIDENCE_THR  = 0.75f;  // tune after testing

// ── Inference stride ─────────────────────────────────────────
// How often KWS runs inference — every N new samples.
// 160 = 10ms at 16kHz, matches SLU frame stride.
// Smaller = more responsive but more CPU load.
constexpr int   KWS_STRIDE_SAMPLES  = 160;

// ── I2S pin config ────────────────────────────────────────────
#define KWS_I2S_PORT      I2S_NUM_0
#define KWS_I2S_BCK_PIN   27
#define KWS_I2S_WS_PIN    33
#define KWS_I2S_DATA_PIN  32

// ── I2S audio format ─────────────────────────────────────────
// 32-bit container, 24-bit mic data in MSB.
// Right-shift 8 → effective int16 range.
// Cast to float with NO normalisation — raw int16-range values.
// This matches the dummy_input[] format used during training.
#define KWS_I2S_BITS      I2S_BITS_PER_SAMPLE_32BIT
#define KWS_RAW_SHIFT     8

// ============================================================
// END MODEL PARAMS
// ============================================================


// ── Ring buffer — declared in MedReminder.ino ─────────────────
// Defined as:
//   int16_t  audio_ring_buf[AUDIO_RING_SIZE];  // 16000 samples
//   int      audio_ring_head;                  // write index
//   bool     audio_ring_full;                  // true after first wrap
// Declared extern here so kws.h and slu.h can both access them
extern int16_t audio_ring_buf[];
extern int     audio_ring_head;
extern bool    audio_ring_full;
constexpr int  AUDIO_RING_SIZE = 16000;        // 1s at 16kHz

// ── KWS flat buffer (linearised ring for inference) ───────────
// TODO: replace size with EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
//       once Edge Impulse library is included
static float   kws_flat[16000];  // TODO: EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE

// ── State ─────────────────────────────────────────────────────
static bool    _kwsReady         = false;
static int     _kwsSamplesSince  = 0;  // samples since last inference

// ============================================================
// INTERNAL HELPERS
// ============================================================

static bool _initI2S() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = 16000,
        .bits_per_sample      = KWS_I2S_BITS,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,    // larger DMA queue for continuous capture
        .dma_buf_len          = 160,  // matches KWS_STRIDE_SAMPLES (10ms)
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };
    i2s_pin_config_t pins = {
        .bck_io_num   = KWS_I2S_BCK_PIN,
        .ws_io_num    = KWS_I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = KWS_I2S_DATA_PIN
    };
    if (i2s_driver_install(KWS_I2S_PORT, &cfg, 0, NULL) != ESP_OK) return false;
    if (i2s_set_pin(KWS_I2S_PORT, &pins)                != ESP_OK) return false;
    return true;
}

/*
 * _readNewSamples()
 * Reads up to KWS_STRIDE_SAMPLES new int16 samples from I2S
 * and writes them into the ring buffer, advancing ring_head.
 * Returns number of samples actually written.
 */
static int _readNewSamples() {
    static int32_t raw[KWS_STRIDE_SAMPLES];
    size_t bytes_read = 0;

    esp_err_t res = i2s_read(KWS_I2S_PORT,
                             raw,
                             KWS_STRIDE_SAMPLES * sizeof(int32_t),
                             &bytes_read,
                             0); // non-blocking — return whatever is ready
    if (res != ESP_OK) return 0;

    int n = (int)(bytes_read / sizeof(int32_t));
    for (int i = 0; i < n; i++) {
        // 24-bit mic in 32-bit word → shift to int16 range, no normalisation
        audio_ring_buf[audio_ring_head] = (int16_t)(raw[i] >> KWS_RAW_SHIFT);
        audio_ring_head = (audio_ring_head + 1) % AUDIO_RING_SIZE;
        if (audio_ring_head == 0) audio_ring_full = true;
    }
    return n;
}

/*
 * _lineariseRingBuffer()
 * Copies ring buffer contents into kws_flat[] in chronological order.
 * Oldest sample first, newest sample last.
 * Only called when ring_full — before that we don't have a full 1s window.
 */
static void _lineariseRingBuffer() {
    // ring_head points to the oldest sample (next write position)
    int oldest = audio_ring_head;
    for (int i = 0; i < AUDIO_RING_SIZE; i++) {
        int idx = (oldest + i) % AUDIO_RING_SIZE;
        // Raw int16-range float — no normalisation, matches training data
        kws_flat[i] = (float)audio_ring_buf[idx];
    }
}

// ============================================================
// PUBLIC API
// ============================================================

void initKWS() {
    if (!_initI2S()) {
        Serial.println("[KWS] I2S init failed");
        return;
    }
    _kwsReady = true;
    Serial.printf("[KWS] Ready — positive label: \"%s\", threshold: %.2f\n",
                  KWS_POSITIVE_LABEL, KWS_CONFIDENCE_THR);
    Serial.printf("[KWS] Ring buffer: %d samples (%.1fs), stride: %dms\n",
                  AUDIO_RING_SIZE,
                  AUDIO_RING_SIZE / 16000.0f,
                  KWS_STRIDE_SAMPLES / 16);
    Serial.println("[KWS] Model placeholder active — "
                   "uncomment Edge Impulse sections when library is installed");
}

/*
 * runKWSInference()
 * Called every loop() tick.
 *
 * 1. Reads KWS_STRIDE_SAMPLES new samples into ring buffer
 * 2. Every KWS_STRIDE_SAMPLES new samples, runs KWS inference
 * 3. Returns true if valid command detected above threshold
 *    → caller should immediately pass ring buffer to SLU
 *    → ring buffer is NOT cleared — SLU reads same contents
 */
bool runKWSInference() {
    if (!_kwsReady) return false;

    // Step 1: read new samples into ring buffer
    int n = _readNewSamples();
    if (n == 0) return false;

    _kwsSamplesSince += n;

    // Only run inference every KWS_STRIDE_SAMPLES new samples
    if (_kwsSamplesSince < KWS_STRIDE_SAMPLES) return false;
    _kwsSamplesSince = 0;

    // Need a full 1s window before we can run
    if (!audio_ring_full) return false;

    // Step 2: linearise ring buffer into flat float array
    _lineariseRingBuffer();

    // ── MVP stub — remove when model library is included ─────
    // TODO: remove this return statement and uncomment block below
    return false;

    // ── Step 3: wrap in signal_t ─────────────────────────────
    // TODO: uncomment when Edge Impulse library is included
    // signal_t signal;
    // if (numpy::signal_from_buffer(kws_flat,
    //         EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal) != 0) {
    //     Serial.println("[KWS] signal_from_buffer failed");
    //     return false;
    // }

    // ── Step 4: run classifier ───────────────────────────────
    // TODO: uncomment when Edge Impulse library is included
    // ei_impulse_result_t result;
    // EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    // if (err != EI_IMPULSE_OK) {
    //     Serial.printf("[KWS] run_classifier error: %d\n", err);
    //     return false;
    // }

    // ── Step 5: find positive label score ────────────────────
    // TODO: uncomment when Edge Impulse library is included
    // float positive_score = 0.0f;
    // for (int i = 0; i < (int)EI_CLASSIFIER_LABEL_COUNT; i++) {
    //     if (strcmp(result.classification[i].label,
    //                KWS_POSITIVE_LABEL) == 0) {
    //         positive_score = result.classification[i].value;
    //         break;
    //     }
    // }
    // Serial.printf("[KWS] command score: %.3f (thr: %.2f)\n",
    //               positive_score, KWS_CONFIDENCE_THR);
    // return (positive_score >= KWS_CONFIDENCE_THR);
}
