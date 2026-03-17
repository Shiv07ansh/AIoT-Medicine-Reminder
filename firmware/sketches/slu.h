#pragma once
/*
 * slu.h
 * ─────────────────────────────────────────────────────────────
 * SLU (Spoken Language Understanding) — Intent Classification
 *
 * PIPELINE:
 *   Shared ring buffer (filled and frozen by KWS)
 *    ↓
 *   Linearise ring buffer → int16[16000] (chronological order)
 *    ↓
 *   Pre-emphasis filter
 *    ↓
 *   Frame into 98 overlapping windows (480 samples, stride 160)
 *    ↓
 *   Hamming window + FFT per frame → power spectrum
 *    ↓
 *   40 mel filterbanks → log energy → 98×40 float
 *    ↓
 *   Average pairs of frames → 49×40 float
 *    ↓
 *   Quantize float → int8 (scale + zero_point from model)
 *    ↓
 *   SLU TFLite model (int8 input [1,49,40], int8 output [8])
 *    ↓
 *   Dequantize int8 → float scores
 *    ↓
 *   Argmax + confidence gate → intent index
 *
 * AUDIO SOURCE:
 *   SLU does NOT capture its own audio.
 *   It reads the shared ring buffer declared in MedReminder.ino,
 *   which was filled by KWS and frozen at trigger time.
 *   This guarantees the utterance that triggered KWS is fully
 *   present in the SLU input window — no gap, no missed audio.
 *
 * ── HOW TO PLUG IN YOUR MODEL ────────────────────────────────
 * 1. Drop quantized model header next to this file
 * 2. Uncomment the #include and update the array name
 * 3. Fill MODEL PARAMS from Python:
 *      interpreter.get_input_details()[0]['quantization']
 *      interpreter.get_output_details()[0]['quantization']
 *    Or just boot and read the Serial printout from initSLU()
 * 4. Uncomment TODO blocks in initSLU()
 * 5. Verify op list in resolver against Netron
 *
 * Public API:
 *   void initSLU()
 *   bool runSLUInference(int* outIntentIdx, float* outScore)
 *     — reads from shared ring buffer (already filled by KWS)
 *     — returns true when result is ready
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <arduinoFFT.h>
// Install: Sketch → Include Library → Manage Libraries
// → search "arduinoFFT" by Enrique Condes

#include <tflm_esp32.h>
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "labels.h"

// ── Model header placeholder ──────────────────────────────────
// TODO: replace with your actual quantized model header, e.g.:
// #include "slu_model_int8.h"
// Array must be declared as:
//   alignas(8) const unsigned char slu_model_int8[] = { ... };

// ── Shared ring buffer — declared in MedReminder.ino ─────────
extern int16_t audio_ring_buf[];
extern int     audio_ring_head;
extern bool    audio_ring_full;
constexpr int  SLU_RING_SIZE = 16000; // must match AUDIO_RING_SIZE in kws.h

// ============================================================
// MODEL PARAMS — edit this section to match your model
// ============================================================

// ── MFE / audio feature extraction ───────────────────────────
constexpr int   SLU_SAMPLE_RATE    = 16000;
constexpr int   SLU_FRAME_LEN      = 480;    // 30ms @ 16kHz
constexpr int   SLU_FRAME_STRIDE   = 160;    // 10ms @ 16kHz
constexpr int   SLU_NUM_FILTERS    = 40;
constexpr int   SLU_NUM_FRAMES_RAW = 98;     // 1 + floor((16000-480)/160)
constexpr int   SLU_NUM_FRAMES     = 49;     // after averaging pairs
constexpr int   SLU_FLAT_SIZE      = SLU_NUM_FRAMES * SLU_NUM_FILTERS; // 1960

// ── Mel frequency range ───────────────────────────────────────
// TODO: confirm against your Edge Impulse project settings
constexpr float SLU_MEL_LOW_HZ    = 0.0f;
constexpr float SLU_MEL_HIGH_HZ   = 8000.0f;

// ── Pre-emphasis ──────────────────────────────────────────────
// Edge Impulse default = 0.97f. Set 0.0f to disable.
constexpr float SLU_PRE_EMPHASIS  = 0.97f;   // TODO: confirm or disable

// ── Input tensor quantization ─────────────────────────────────
// Get from: interpreter.get_input_details()[0]['quantization']
// Or read from Serial output of initSLU() after first boot.
constexpr float SLU_INP_SCALE      = 1.0f;   // TODO: replace
constexpr int   SLU_INP_ZERO_POINT = 0;      // TODO: replace

// ── Output tensor quantization ────────────────────────────────
// Get from: interpreter.get_output_details()[0]['quantization']
constexpr float SLU_OUT_SCALE      = 1.0f;   // TODO: replace
constexpr int   SLU_OUT_ZERO_POINT = 0;      // TODO: replace

// ── Confidence threshold ──────────────────────────────────────
constexpr float SLU_CONFIDENCE_THR = 0.60f;  // tune after testing

// ── TFLite arena ─────────────────────────────────────────────
// Increase if AllocateTensors() fails.
// Read "[SLU] Arena used: X bytes" on Serial to tune downward.
constexpr int   SLU_ARENA_SIZE     = 64 * 1024;

// ── FFT size (power of 2, must be >= SLU_FRAME_LEN=480) ──────
constexpr int   SLU_FFT_SIZE       = 512;

// ============================================================
// END MODEL PARAMS
// ============================================================


// ── Static allocations ───────────────────────────────────────
static uint8_t*                  slu_arena_buf   = nullptr;
static tflite::MicroInterpreter* slu_interpreter = nullptr;
static const tflite::Model*      slu_model_ptr   = nullptr;

// Linearised copy of ring buffer (int16, chronological)
static int16_t slu_audio_linear[SLU_RING_SIZE];

// MFE working buffers
static float   slu_mfe_raw[SLU_NUM_FRAMES_RAW][SLU_NUM_FILTERS]; // 98×40
static float   slu_mfe_ds [SLU_NUM_FRAMES][SLU_NUM_FILTERS];      // 49×40
static int8_t  slu_mfe_q  [SLU_FLAT_SIZE];                        // 1960

// FFT buffers
static double  slu_fft_real[SLU_FFT_SIZE];
static double  slu_fft_imag[SLU_FFT_SIZE];

static ArduinoFFT<double> slu_fft =
    ArduinoFFT<double>(slu_fft_real, slu_fft_imag,
                       SLU_FFT_SIZE, (double)SLU_SAMPLE_RATE);

// ============================================================
// INTERNAL HELPERS
// ============================================================

static inline float _hzToMel(float hz) {
    return 2595.0f * log10f(1.0f + hz / 700.0f);
}
static inline float _melToHz(float mel) {
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

/*
 * _lineariseRingBuffer()
 * Copies ring buffer into slu_audio_linear[] in chronological order.
 * Oldest sample first (at ring_head), newest sample last.
 * Result is a flat int16 array ready for MFE processing.
 */
static void _lineariseRingBuffer() {
    int oldest = audio_ring_head; // oldest = next write position
    for (int i = 0; i < SLU_RING_SIZE; i++) {
        slu_audio_linear[i] = audio_ring_buf[(oldest + i) % SLU_RING_SIZE];
    }
}

/*
 * _applyMelFilters()
 * Triangular mel filterbank on one-sided power spectrum.
 * Filter boundaries computed on the fly.
 * Output: log10 energy per filter bank.
 */
static void _applyMelFilters(const double* power, int n_bins, float* out) {
    float mel_lo = _hzToMel(SLU_MEL_LOW_HZ);
    float mel_hi = _hzToMel(SLU_MEL_HIGH_HZ);

    float mel_pts[SLU_NUM_FILTERS + 2];
    for (int i = 0; i < SLU_NUM_FILTERS + 2; i++) {
        mel_pts[i] = mel_lo + i * (mel_hi - mel_lo) /
                     (float)(SLU_NUM_FILTERS + 1);
    }

    float bin_pts[SLU_NUM_FILTERS + 2];
    for (int i = 0; i < SLU_NUM_FILTERS + 2; i++) {
        bin_pts[i] = (_melToHz(mel_pts[i]) / SLU_MEL_HIGH_HZ) *
                     (float)(n_bins - 1);
    }

    for (int f = 0; f < SLU_NUM_FILTERS; f++) {
        float sum = 0.0f;
        int   lo  = (int)bin_pts[f];
        int   mid = (int)bin_pts[f + 1];
        int   hi  = (int)bin_pts[f + 2];

        for (int k = lo; k < mid && k < n_bins; k++) {
            float w = (k - bin_pts[f]) /
                      (bin_pts[f+1] - bin_pts[f] + 1e-8f);
            sum += (float)power[k] * w;
        }
        for (int k = mid; k <= hi && k < n_bins; k++) {
            float w = (bin_pts[f+2] - k) /
                      (bin_pts[f+2] - bin_pts[f+1] + 1e-8f);
            sum += (float)power[k] * w;
        }
        out[f] = log10f(sum + 1e-10f);
    }
}

/*
 * _computeMFE()
 * Full feature extraction on linearised int16 audio:
 *   pre-emphasis → 98 frames → hamming+FFT → mel → 98×40
 *   → average pairs → 49×40 → quantize → int8[1960]
 */
static void _computeMFE(const int16_t* audio, int8_t* out_flat) {
    int n_bins = SLU_FFT_SIZE / 2;

    for (int fr = 0; fr < SLU_NUM_FRAMES_RAW; fr++) {
        int offset = fr * SLU_FRAME_STRIDE;

        for (int i = 0; i < SLU_FFT_SIZE; i++) {
            float sample = 0.0f;
            if (i < SLU_FRAME_LEN && (offset + i) < SLU_RING_SIZE) {
                float cur  = (float)audio[offset + i];
                float prev = (i > 0)
                             ? (float)audio[offset + i - 1]
                             : (offset > 0 ? (float)audio[offset-1] : 0.0f);
                sample = cur - SLU_PRE_EMPHASIS * prev;
            }
            float w = 0.54f - 0.46f * cosf(2.0f * M_PI * i /
                                            (float)(SLU_FFT_SIZE - 1));
            slu_fft_real[i] = (double)(sample * w);
            slu_fft_imag[i] = 0.0;
        }

        slu_fft.compute(FFTDirection::Forward);
        slu_fft.complexToMagnitude();

        double power[SLU_FFT_SIZE / 2];
        for (int k = 0; k < n_bins; k++) {
            power[k] = slu_fft_real[k] * slu_fft_real[k];
        }

        _applyMelFilters(power, n_bins, slu_mfe_raw[fr]);
    }

    // Average pairs: 98 → 49
    for (int f = 0; f < SLU_NUM_FRAMES; f++) {
        for (int c = 0; c < SLU_NUM_FILTERS; c++) {
            slu_mfe_ds[f][c] = (slu_mfe_raw[f*2][c] +
                                 slu_mfe_raw[f*2+1][c]) * 0.5f;
        }
    }

    // Quantize float → int8
    for (int i = 0; i < SLU_NUM_FRAMES; i++) {
        for (int j = 0; j < SLU_NUM_FILTERS; j++) {
            int q = (int)roundf(slu_mfe_ds[i][j] / SLU_INP_SCALE)
                    + SLU_INP_ZERO_POINT;
            q = q < -128 ? -128 : (q > 127 ? 127 : q);
            out_flat[i * SLU_NUM_FILTERS + j] = (int8_t)q;
        }
    }
}

// ============================================================
// PUBLIC API
// ============================================================

void initSLU() {
    slu_arena_buf = (uint8_t*)malloc(SLU_ARENA_SIZE);
    if (!slu_arena_buf) {
        Serial.printf("[SLU] malloc failed — need %d bytes\n", SLU_ARENA_SIZE);
        return;
    }

    // ── Load model ───────────────────────────────────────────
    // TODO: uncomment when model header is available
    // slu_model_ptr = tflite::GetModel(slu_model_int8);
    // if (!slu_model_ptr ||
    //     slu_model_ptr->version() != TFLITE_SCHEMA_VERSION) {
    //     Serial.println("[SLU] Model invalid or schema mismatch");
    //     return;
    // }

    // ── Op resolver ──────────────────────────────────────────
    // TODO: verify op list
    static tflite::MicroMutableOpResolver<10> resolver;
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddReshape();
    resolver.AddQuantize();
    resolver.AddDequantize();
    resolver.AddMean();
    resolver.AddPad();
    resolver.AddMaxPool2D();

    // ── Create interpreter ───────────────────────────────────
    // TODO: uncomment when model header is available
    // static tflite::MicroInterpreter interp(
    //     slu_model_ptr, resolver, slu_arena_buf, SLU_ARENA_SIZE);
    // slu_interpreter = &interp;
    // if (slu_interpreter->AllocateTensors() != kTfLiteOk) {
    //     Serial.println("[SLU] AllocateTensors failed — "
    //                    "increase SLU_ARENA_SIZE");
    //     return;
    // }

    // ── Print tensor info ────────────────────────────────────
    // Uncomment after first boot to read quantization params.
    // Copy printed values into SLU_INP_SCALE / SLU_INP_ZERO_POINT above.
    // TODO: uncomment when model is loaded
    // TfLiteTensor* inp = slu_interpreter->input(0);
    // TfLiteTensor* out = slu_interpreter->output(0);
    // Serial.printf("[SLU] Arena used:   %u / %d bytes\n",
    //               (unsigned)slu_interpreter->arena_used_bytes(),
    //               SLU_ARENA_SIZE);
    // Serial.printf("[SLU] Input  shape: [%d, %d, %d]\n",
    //               inp->dims->data[1], inp->dims->data[2],
    //               inp->dims->data[3]);
    // Serial.printf("[SLU] Input  type:  %d  (9=int8)\n", inp->type);
    // Serial.printf("[SLU] Input  quant: scale=%.6f  zp=%d\n",
    //               inp->params.scale, inp->params.zero_point);
    // Serial.printf("[SLU] Output quant: scale=%.6f  zp=%d\n",
    //               out->params.scale, out->params.zero_point);

    Serial.println("[SLU] Init done — model placeholder active");
}

/*
 * runSLUInference()
 * Reads shared ring buffer (already filled + frozen by KWS),
 * runs full MFE pipeline, runs TFLite inference.
 * Returns true when result is ready.
 */
bool runSLUInference(int* outIntentIdx, float* outScore) {

    // ── MVP stub — remove when model is loaded ───────────────
    if (!slu_interpreter) {
        Serial.println("[SLU] No model — returning IRRELEVANT");
        *outIntentIdx = INTENT_IRRELEVANT;
        *outScore     = 0.0f;
        return true;
    }

    // ── 1. Linearise ring buffer ─────────────────────────────
    // Ring buffer was filled by KWS and contains the triggering utterance.
    // Oldest sample first, newest last — same order as training data.
    _lineariseRingBuffer();

    // ── 2. MFE feature extraction + quantization ─────────────
    _computeMFE(slu_audio_linear, slu_mfe_q);

    // ── 3. Copy int8 features into input tensor ───────────────
    memcpy(slu_interpreter->input(0)->data.int8,
           slu_mfe_q, SLU_FLAT_SIZE * sizeof(int8_t));

    // ── 4. Inference ─────────────────────────────────────────
    if (slu_interpreter->Invoke() != kTfLiteOk) {
        Serial.println("[SLU] Invoke failed");
        return false;
    }

    // ── 5. Dequantize output + argmax ────────────────────────
    TfLiteTensor* out = slu_interpreter->output(0);
    int   maxIdx   = 0;
    float maxScore = -1e9f;

    for (int i = 0; i < NUM_CLASSES; i++) {
        float score = ((float)out->data.int8[i] -
                       (float)SLU_OUT_ZERO_POINT) * SLU_OUT_SCALE;
        Serial.printf("[SLU] %s: %.4f\n", class_labels[i], score);
        if (score > maxScore) { maxScore = score; maxIdx = i; }
    }
    Serial.printf("[SLU] Top: %s (%.4f)\n", class_labels[maxIdx], maxScore);

    // ── 6. Confidence gate ───────────────────────────────────
    if (maxScore < SLU_CONFIDENCE_THR) {
        Serial.printf("[SLU] Below threshold %.2f → IRRELEVANT\n",
                      SLU_CONFIDENCE_THR);
        *outIntentIdx = INTENT_IRRELEVANT;
        *outScore     = maxScore;
        return true;
    }

    *outIntentIdx = maxIdx;
    *outScore     = maxScore;
    return true;
}
