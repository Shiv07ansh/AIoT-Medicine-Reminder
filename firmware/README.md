# MedReminder — System Architecture & Program Flow
### ESP32-S3 AloT Medicine Reminder System

---

## Overview

MedReminder is an always-on voice-aware medicine reminder system running on an ESP32-S3. It continuously listens for voice commands using a KWS (Keyword Spotting) model, classifies intent using an SLU (Spoken Language Understanding) model, cross-verifies physical medicine-taking with two sensors, and triggers appropriate responses including OLED display, buzzer, email, and dashboard logging.

There is no wakeword. The system is always listening. KWS acts as a binary gate — valid command vs noise — and when it fires, the same audio is immediately passed to SLU for intent classification. No audio is ever re-captured or lost.

---

## File Map

```
MedReminder/
├── MedReminder.ino     — Entry point: ring buffer, state machine, triggers
├── kws.h               — I2S driver, ring buffer writer, KWS inference
├── slu.h               — MFE pipeline, TFLite inference, intent output
├── sensors.h           — OLED, buzzer, LED, hall sensor, ultrasonic
├── connectivity.h      — WiFi, SMTP email, HTTP dashboard
├── credentials.h       — Secrets: WiFi SSID, SMTP, dashboard URL
└── labels.h            — Intent index #defines and class_labels[]
```

### Ownership and data flow between files

```
credentials.h  ←── read by ──→  connectivity.h
labels.h       ←── read by ──→  slu.h, MedReminder.ino

MedReminder.ino
  declares: audio_ring_buf[], audio_ring_head, audio_ring_full
       ↓ extern'd into
  kws.h    — writes new I2S samples into ring buffer every loop() tick
  slu.h    — reads ring buffer (read-only) when KWS triggers

MedReminder.ino calls:
  initSensors()      → sensors.h
  initWiFi()         → connectivity.h
  initKWS()          → kws.h  (also installs I2S driver)
  initSLU()          → slu.h  (allocates TFLite arena)
  runKWSInference()  → kws.h
  runSLUInference()  → slu.h
  showOnOLED()       → sensors.h
  buzz()             → sensors.h
  setLED()           → sensors.h
  readHallSensor()   → sensors.h
  readHandPresent()  → sensors.h
  readDistanceCM()   → sensors.h
  sendEmail()        → connectivity.h
  postToDashboard()  → connectivity.h
```

---

## Boot Sequence — `setup()`

Executed once on power-on. Order matters — `initKWS()` must run before `initSLU()` because KWS installs the I2S driver that SLU also depends on.

```
1. Serial.begin(115200)
2. memset(audio_ring_buf, 0)        — zero ring buffer before first use
   audio_ring_head = 0
   audio_ring_full = false

3. initSensors()                    — sensors.h
   └── Wire.begin(SDA=21, SCL=22)
   └── SSD1306 OLED init → display "Booting..."
   └── pinMode: buzzer (GPIO15), LED (GPIO2)
   └── pinMode: HC-SR04 TRIG (GPIO25), ECHO (GPIO26)
   └── pinMode: hall sensor (GPIO34, INPUT_PULLUP)

4. initWiFi()                       — connectivity.h
   └── WiFi.begin(SSID, PASS)
   └── wifiClientSecure.setInsecure()

5. initKWS()                        — kws.h
   └── i2s_driver_install(I2S_NUM_0)
       sample_rate=16000, bits=32, DMA buf=160 samples (10ms)
   └── i2s_set_pin(BCK=27, WS=33, DATA=32)
   └── _kwsReady = true

6. initSLU()                        — slu.h
   └── malloc(64KB) → slu_arena_buf
   └── [TODO] tflite::GetModel(slu_model_int8)
   └── [TODO] MicroInterpreter::AllocateTensors()
   └── [TODO] print input/output tensor quant params to Serial

7. Wire.begin() + rtc.begin()       — DS3231 RTC
   └── if lostPower: rtc.adjust to compile time

8. SPI.begin() + SD.begin()
   └── create /log.csv if not exists

9. enterState(STATE_IDLE)
   └── OLED: "Ready / Listening..."
   └── LED off
```

---

## The Ring Buffer — Shared Audio Memory

The ring buffer is the architectural centrepiece of the audio pipeline. It is declared in `MedReminder.ino` and `extern`'d into both `kws.h` and `slu.h`.

```
int16_t audio_ring_buf[16000]   — 16000 samples = exactly 1 second at 16kHz
int     audio_ring_head         — index of next write position (= oldest sample)
bool    audio_ring_full         — true after first complete 1s fill
```

**How writing works (kws.h):**
Every `loop()` tick, `_readNewSamples()` pulls 160 new int16 samples from the I2S DMA buffer and writes them into `audio_ring_buf` starting at `audio_ring_head`, then advances the head. When `audio_ring_head` wraps past 15999 back to 0, `audio_ring_full` is set true.

```
Before wrap:   [0][1][2]...[head][empty]...[15999]
                             ↑ next write

After 1 full second:
               [oldest]..........[newest]
                  ↑ ring_head points here (oldest = next overwrite)
```

**Why this design:**
When KWS detects a command, the utterance that triggered it is already inside the ring buffer — it was captured in real time as the user spoke. The ring buffer is not cleared. SLU immediately reads the same 1 second of audio and processes it. This is how commercial always-on voice systems work. A naive design that captured fresh audio after KWS detection would always miss the triggering utterance.

**How reading works (slu.h):**
`_lineariseRingBuffer()` copies the ring buffer into a flat `int16` array in chronological order — oldest sample first, newest sample last — by starting the copy at `audio_ring_head` and wrapping around.

```
Flat output: [oldest sample ... ... ... ... newest sample]
              ↑ audio[0]                    ↑ audio[15999]
```

---

## Main Loop — `loop()`

`loop()` is a simple state machine dispatcher. It runs as fast as the ESP32 allows — typically thousands of times per second, throttled in practice by the I2S read in `runKWSInference()` which blocks for 10ms per call.

```
loop()
  └── switch(currentState)
        STATE_IDLE            → handleIdle()
        STATE_ALARM_RINGING   → handleAlarmRinging()
        STATE_AWAIT_SLU       → handleAwaitSLU()
        STATE_AWAIT_LID       → handleAwaitLid()
        STATE_AWAIT_HAND      → handleAwaitHand()
        STATE_EVENT_CONFIRMED → handleEventConfirmed()
        STATE_REMIND_LATER    → handleRemindLater()
        STATE_SOS             → handleSOS()
```

---

## State Machine — All States

### STATE_IDLE
**Entry effects:** OLED "Ready / Listening...", LED off

**Every tick:**
```
1. Check Serial for 'A'/'a'  →  if received: enterState(ALARM_RINGING)

2. runKWSInference()          →  kws.h
   ├── _readNewSamples()
   │     i2s_read(I2S_NUM_0, 160 samples, non-blocking)
   │     for each sample: int16 = int32 >> 8   (24-bit mic, no normalisation)
   │     write into audio_ring_buf[ring_head]
   │     advance ring_head, set ring_full on first wrap
   │
   ├── _kwsSamplesSince += n
   ├── if _kwsSamplesSince < 160: return false  (wait for full stride)
   ├── if !audio_ring_full:       return false  (wait for 1s fill)
   │
   ├── _lineariseRingBuffer()     →  kws_flat[] (float, raw int16-range)
   ├── [TODO] numpy::signal_from_buffer(kws_flat)
   ├── [TODO] run_classifier() → binary: "command" vs "noise"
   ├── [TODO] positive_score >= KWS_CONFIDENCE_THR (0.75)?
   │         YES → return true
   │         NO  → return false
   │
   └── MVP stub: always returns false

   if true:
     sluCallerState = STATE_IDLE
     enterState(STATE_AWAIT_SLU)
```

---

### STATE_ALARM_RINGING
**Entry effects:** OLED "Time for meds!", buzzer x3, LED on

**Every tick:**
```
1. millis() - lastBuzz > 3000ms?  →  buzz(150ms) reminder pulse

2. runKWSInference()              →  same as IDLE
   if true:
     sluCallerState = STATE_ALARM_RINGING
     enterState(STATE_AWAIT_SLU)
```

The `sluCallerState` is recorded so that after SLU finishes, if the intent is irrelevant, the system returns to ALARM_RINGING rather than IDLE — the alarm keeps going until the user gives a valid response.

---

### STATE_AWAIT_SLU
**Entry effects:** OLED "Processing..."

Ring buffer is frozen at the moment KWS triggered. SLU processes it immediately.

**Every tick:**
```
1. millis() - stateEnteredAt > 5000ms?
   YES → timeout → return to sluCallerState (ALARM_RINGING or IDLE)

2. runSLUInference(&intentIdx, &intentScore)   →  slu.h

   ├── _lineariseRingBuffer()
   │     copies audio_ring_buf[] → slu_audio_linear[] chronologically
   │
   ├── _computeMFE(slu_audio_linear, slu_mfe_q)
   │   │
   │   ├── PRE-EMPHASIS (per sample)
   │   │     y[n] = x[n] - 0.97 * x[n-1]
   │   │
   │   ├── FRAME LOOP (98 frames)
   │   │     offset = frame * 160  (10ms stride)
   │   │     for i in 0..511:
   │   │       sample = pre-emphasised audio[offset+i]  (zero-pad if i >= 480)
   │   │       hamming = 0.54 - 0.46 * cos(2π*i/511)
   │   │       fft_real[i] = sample * hamming
   │   │       fft_imag[i] = 0
   │   │     FFT → complex spectrum
   │   │     complexToMagnitude() → fft_real[] = magnitude
   │   │     power[k] = fft_real[k]²  (256 one-sided bins)
   │   │
   │   │     _applyMelFilters(power → slu_mfe_raw[frame][0..39])
   │   │       40 triangular filters on mel scale 0–8000Hz
   │   │       each filter: weighted sum over power bins
   │   │       log10(sum + 1e-10) per filter
   │   │       → slu_mfe_raw[98][40]
   │   │
   │   ├── DOWNSAMPLE 98 → 49
   │   │     slu_mfe_ds[f][c] = mean(slu_mfe_raw[f*2][c], slu_mfe_raw[f*2+1][c])
   │   │     → slu_mfe_ds[49][40]
   │   │
   │   └── QUANTIZE float → int8
   │         q = round(value / SLU_INP_SCALE) + SLU_INP_ZERO_POINT
   │         clamp to [-128, 127]
   │         → slu_mfe_q[1960]
   │
   ├── memcpy(input_tensor->data.int8, slu_mfe_q, 1960)
   │
   ├── TFLite Invoke()
   │     model input:  int8[1, 49, 40]
   │     model output: int8[8]  — one score per intent class
   │
   ├── DEQUANTIZE output
   │     score[i] = (output->data.int8[i] - SLU_OUT_ZERO_POINT) * SLU_OUT_SCALE
   │
   ├── ARGMAX → maxIdx, maxScore
   │
   ├── maxScore < SLU_CONFIDENCE_THR (0.60)?
   │     YES → intentIdx = INTENT_IRRELEVANT
   │     NO  → intentIdx = maxIdx
   │
   └── return true

   if true:
     lastIntent = intentIdx
     lastIntentScore = intentScore
     dispatchIntent(intentIdx, intentScore)
```

---

### INTENT DISPATCH — `dispatchIntent()`

Called immediately after SLU returns a result. Routes to the appropriate state and trigger function.

```
INTENT_CONFIRM_TAKEN    →  enterState(AWAIT_LID)
INTENT_DENY_TAKEN       →  triggerDenyTaken() + enterState(REMIND_LATER)
INTENT_REMIND_LATER     →  triggerRemindLater() + enterState(REMIND_LATER)
INTENT_ASK_TIME         →  triggerAskTime() + enterState(IDLE)
INTENT_ASK_MED_DETAILS  →  triggerAskMedDetails() + enterState(IDLE)
INTENT_ASK_SCHEDULE     →  triggerAskSchedule() + enterState(IDLE)
INTENT_NOTIFY_SOS       →  enterState(SOS)
INTENT_IRRELEVANT       →  triggerIrrelevant()
                           + enterState(sluCallerState)  ← back to origin
```

---

### STATE_AWAIT_LID
**Entry effects:** OLED "Open medicine box please"

Physical verification step 1. Confirms the user actually opened the medicine box.

**Every tick:**
```
millis() - stateEnteredAt > 5000ms?
  YES → OLED "Lid not opened" → enterState(REMIND_LATER)

readHallSensor()   →  sensors.h
  digitalRead(PIN_HALL=34) == HIGH?  (magnet gone = lid open)
  YES → enterState(AWAIT_HAND)
  NO  → keep polling
```

---

### STATE_AWAIT_HAND
**Entry effects:** OLED "Take your medicine"

Physical verification step 2. Confirms the user reached into the box.

**Every tick:**
```
millis() - stateEnteredAt > 5000ms?
  YES → OLED "Hand not detected" → enterState(REMIND_LATER)

readHandPresent()   →  sensors.h
  readDistanceCM()
    HC-SR04: TRIG pulse → measure ECHO duration → cm = duration * 0.0343 / 2
  distanceMM = cm * 10
  distanceMM < D_EMPTY_MM - 10mm?   (hand closer than empty-box baseline)
  YES → enterState(EVENT_CONFIRMED)
  NO  → keep polling
```

---

### STATE_EVENT_CONFIRMED
**Entry effects:** OLED "Confirmed! / Medicine taken", buzz(500ms), LED off

All three conditions met: voice intent + lid open + hand present.

**Runs once then returns to IDLE:**
```
rtc.now()                              — get timestamp

logToSD("confirm_taken", score, now)   — sensors.h + SD
  → append CSV row: timestamp, label, confidence, distance_cm, hall

sendEmail("Medicine Taken", body)      — connectivity.h
  → raw TLS SMTP to caregiver address

postToDashboard("confirm_taken", score) — connectivity.h
  → HTTP POST JSON to DASHBOARD_URL

triggerConfirmTaken()
  → Serial log
  → OLED "Well done! / Dose logged"
  → [TODO] TTS playback
  → [TODO] green LED blink

delay(3000)
enterState(STATE_IDLE)
```

---

### STATE_REMIND_LATER
**Entry effects:** OLED "Reminding in 5 minutes...", LED off, timer started

**Every tick:**
```
millis() - remindLaterAt >= 300000ms (5 min)?
  YES → enterState(ALARM_RINGING)
  NO  → do nothing, yield back to loop()
```

---

### STATE_SOS
**Entry effects:** OLED "!! SOS !!", buzz(1000ms)

**Runs once then returns to IDLE:**
```
sendEmail("!! SOS ALERT !!", body)     — connectivity.h
postToDashboard("notify_sos", 1.0)     — connectivity.h
logToSD("notify_sos", 1.0, now)        — SD card
triggerNotifySOS()
  → OLED "SOS SENT / Help coming"
  → buzz x2
  → [TODO] TTS
  → [TODO] GSM SMS
delay(3000)
enterState(STATE_IDLE)
```

---

## Complete Audio Pipeline Summary

```
HARDWARE
  I2S mic (BCK=27, WS=33, DATA=32)
  32-bit DMA words, 24-bit mic data in MSB, 16kHz

RING BUFFER FILL  (kws.h, every loop() tick)
  int32 >> 8  →  int16 range, no normalisation
  written into audio_ring_buf[ring_head], head advances

KWS INFERENCE  (kws.h, every 160 new samples = 10ms)
  _lineariseRingBuffer()  →  kws_flat[] float, oldest→newest
  Edge Impulse DSP internally: MFE feature extraction (float32)
  Binary classifier: "command" vs "noise"
  Threshold: 0.75
  On positive: ring buffer frozen, enterState(AWAIT_SLU)

SLU INFERENCE  (slu.h, called once from handleAwaitSLU())
  _lineariseRingBuffer()  →  slu_audio_linear[] int16, oldest→newest
  Pre-emphasis (0.97)
  98 frames × 480 samples, stride 160
  Hamming window + 512-point FFT per frame
  40-bin mel filterbank, log10 compression  →  98×40 float
  Average pairs  →  49×40 float
  Quantize  →  int8[1960]
  TFLite Invoke: int8[1,49,40]  →  int8[8]
  Dequantize  →  float[8] scores
  Argmax + confidence gate (0.60)  →  intent index 0–7

DISPATCH
  intent index → dispatchIntent() → state transition + trigger function
```

---

## Intent Classes

Defined in `labels.h`. Index must match the SLU model output layer order exactly.

| Index | Label | What happens |
|-------|-------|-------------|
| 0 | `ask_med_details` | OLED shows medicine info, TODO: TTS |
| 1 | `ask_schedule` | OLED shows schedule, TODO: TTS |
| 2 | `ask_time` | OLED shows current RTC time, TODO: TTS |
| 3 | `confirm_taken` | → AWAIT_LID → AWAIT_HAND → EVENT_CONFIRMED |
| 4 | `deny_taken` | → REMIND_LATER (5 min) |
| 5 | `irrelevant` | No action, return to previous state |
| 6 | `notify_sos` | → SOS (email + dashboard + TODO: SMS) |
| 7 | `remind_later` | → REMIND_LATER (5 min) |

---

## Peripheral Pin Summary

| Peripheral | Interface | Pins |
|-----------|-----------|------|
| I2S Microphone | I2S | BCK=27, WS=33, DATA=32 |
| OLED SSD1306 | I2C | SDA=21, SCL=22 |
| RTC DS3231 | I2C | SDA=21, SCL=22 |
| Buzzer | GPIO | 15 |
| Status LED | GPIO | 2 |
| Hall Sensor | GPIO | 34 (INPUT_PULLUP) |
| HC-SR04 Ultrasonic | GPIO | TRIG=25, ECHO=26 |
| SD Card | SPI | SCK=18, MISO=19, MOSI=23, CS=5 |

---

