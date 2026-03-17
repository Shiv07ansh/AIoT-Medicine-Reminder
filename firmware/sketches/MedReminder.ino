/*
 * MedReminder.ino
 * ============================================================
 * AIoT Medicine Reminder System — ESP32-S3
 *
 * FULL PIPELINE:
 *
 *   Continuous I2S stream (16kHz, int16)
 *    ↓
 *   Ring buffer — 16000 samples (1s), always rolling
 *    ↓
 *   KWS runs every 160 samples (10ms stride)
 *   Binary: "valid command" vs "noise/irrelevant"
 *    ↓ (on positive detection)
 *   Ring buffer frozen — contains the triggering utterance
 *    ↓
 *   SLU reads same ring buffer — no re-capture, no gap
 *   MFE → 49×40 → int8 → TFLite → dequantize → intent
 *    ↓
 *   dispatchIntent() → state machine transition + trigger
 *
 * STATE MACHINE:
 *   IDLE              KWS always listening
 *   ALARM_RINGING     Buzzer/LED active, KWS listening
 *   AWAIT_SLU         KWS triggered — run SLU on ring buffer
 *   AWAIT_LID         Waiting for hall sensor (lid open)
 *   AWAIT_HAND        Waiting for ultrasonic (hand in box)
 *   EVENT_CONFIRMED   Log + email + TTS, back to IDLE
 *   REMIND_LATER      5-min delay, re-enter ALARM_RINGING
 *   SOS               Emergency alert, back to IDLE
 *
 * FILE STRUCTURE:
 *   MedReminder.ino   — this file: ring buffer, state machine
 *   kws.h             — KWS model + ring buffer writer
 *   slu.h             — SLU model + MFE pipeline
 *   sensors.h         — OLED, buzzer, LED, hall, ultrasonic
 *   connectivity.h    — WiFi, SMTP email, dashboard POST
 *   credentials.h     — secrets (keep out of git!)
 *   labels.h          — intent index defines + class_labels[]
 * ============================================================
 */

#include <Arduino.h>
#include "credentials.h"
#include "sensors.h"
#include "connectivity.h"
#include "kws.h"
#include "slu.h"
#include "labels.h"

#include "DS3231.h"
RTC_DS3231 rtc;

#include <SPI.h>
#include <SD.h>
#define SD_CS   5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

// ============================================================
// SHARED RING BUFFER
// Declared here as globals. extern'd in kws.h and slu.h.
// kws.h writes new I2S samples into it every loop() tick.
// slu.h reads it (read-only) when KWS triggers.
// ============================================================

int16_t audio_ring_buf[16000]; // 1s at 16kHz — matches AUDIO_RING_SIZE
int     audio_ring_head = 0;   // next write position (oldest sample)
bool    audio_ring_full = false; // true after first complete 1s fill

// ============================================================
// STATE MACHINE
// ============================================================

enum SystemState {
  STATE_IDLE,
  STATE_ALARM_RINGING,
  STATE_AWAIT_SLU,
  STATE_AWAIT_LID,
  STATE_AWAIT_HAND,
  STATE_EVENT_CONFIRMED,
  STATE_REMIND_LATER,
  STATE_SOS
};

SystemState currentState   = STATE_IDLE;
SystemState sluCallerState = STATE_IDLE;

// ── Timers ───────────────────────────────────────────────────
unsigned long stateEnteredAt = 0;
unsigned long remindLaterAt  = 0;

constexpr unsigned long SLU_TIMEOUT_MS  =   5000UL; // 5s no result → back
constexpr unsigned long LID_TIMEOUT_MS  =   5000UL; // 5s no lid    → remind
constexpr unsigned long HAND_TIMEOUT_MS =   5000UL; // 5s no hand   → remind
constexpr unsigned long REMIND_DELAY_MS = 300000UL; // 5min re-alarm

// ── Last SLU result ──────────────────────────────────────────
int   lastIntent      = -1;
float lastIntentScore = 0.0f;

// ── Forward declarations ─────────────────────────────────────
void enterState(SystemState s);
void dispatchIntent(int intentIndex, float score);
void logToSD(const char* label, float confidence, DateTime timestamp);
void triggerConfirmTaken();
void triggerDenyTaken();
void triggerRemindLater();
void triggerAskTime();
void triggerAskMedDetails();
void triggerAskSchedule();
void triggerNotifySOS();
void triggerIrrelevant();

// ============================================================
// STATE ENTRY
// ============================================================

void enterState(SystemState s) {
  currentState   = s;
  stateEnteredAt = millis();

  switch (s) {
    case STATE_IDLE:
      showOnOLED("Ready", "Listening...");
      setLED(false);
      Serial.println("[STATE] IDLE");
      break;

    case STATE_ALARM_RINGING:
      showOnOLED("Time for meds!", "Speak to respond");
      buzz(200); delay(100);
      buzz(200); delay(100);
      buzz(200);
      setLED(true);
      Serial.println("[STATE] ALARM_RINGING");
      break;

    case STATE_AWAIT_SLU:
      showOnOLED("Processing...", "");
      Serial.println("[STATE] AWAIT_SLU");
      break;

    case STATE_AWAIT_LID:
      showOnOLED("Open medicine", "box please");
      Serial.println("[STATE] AWAIT_LID");
      break;

    case STATE_AWAIT_HAND:
      showOnOLED("Take your", "medicine");
      Serial.println("[STATE] AWAIT_HAND");
      break;

    case STATE_EVENT_CONFIRMED:
      showOnOLED("Confirmed!", "Medicine taken");
      buzz(500);
      setLED(false);
      Serial.println("[STATE] EVENT_CONFIRMED");
      break;

    case STATE_REMIND_LATER:
      showOnOLED("Reminding in", "5 minutes...");
      setLED(false);
      remindLaterAt = millis();
      Serial.println("[STATE] REMIND_LATER");
      break;

    case STATE_SOS:
      showOnOLED("!! SOS !!", "Alerting caregiver");
      buzz(1000);
      Serial.println("[STATE] SOS");
      break;
  }
}

// ============================================================
// INTENT DISPATCH
// ============================================================

void dispatchIntent(int intentIndex, float score) {
  Serial.printf("[INTENT] %s (%.3f)\n", class_labels[intentIndex], score);

  switch (intentIndex) {
    case INTENT_CONFIRM_TAKEN:
      enterState(STATE_AWAIT_LID);
      break;

    case INTENT_DENY_TAKEN:
      triggerDenyTaken();
      enterState(STATE_REMIND_LATER);
      break;

    case INTENT_REMIND_LATER:
      triggerRemindLater();
      enterState(STATE_REMIND_LATER);
      break;

    case INTENT_ASK_TIME:
      triggerAskTime();
      enterState(STATE_IDLE);
      break;

    case INTENT_ASK_MED_DETAILS:
      triggerAskMedDetails();
      enterState(STATE_IDLE);
      break;

    case INTENT_ASK_SCHEDULE:
      triggerAskSchedule();
      enterState(STATE_IDLE);
      break;

    case INTENT_NOTIFY_SOS:
      enterState(STATE_SOS);
      break;

    case INTENT_IRRELEVANT:
    default:
      triggerIrrelevant();
      // Return to wherever we came from
      enterState(sluCallerState == STATE_ALARM_RINGING
                   ? STATE_ALARM_RINGING
                   : STATE_IDLE);
      break;
  }
}

// ============================================================
// STATE HANDLERS
// ============================================================

void handleIdle() {
  // MVP: Serial 'A' triggers alarm for testing
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'A' || c == 'a') {
      enterState(STATE_ALARM_RINGING);
      return;
    }
  }

  // KWS reads new I2S samples into ring buffer every call.
  // Returns true when a valid command is detected.
  if (runKWSInference()) {
    Serial.println("[KWS] Command detected (out-of-turn)");
    sluCallerState = STATE_IDLE;
    // Ring buffer now contains the triggering utterance.
    // Go straight to SLU — no re-capture needed.
    enterState(STATE_AWAIT_SLU);
  }
}

void handleAlarmRinging() {
  // Periodic buzz reminder while waiting
  static unsigned long lastBuzz = 0;
  if (millis() - lastBuzz > 3000) {
    buzz(150);
    lastBuzz = millis();
  }

  // KWS still runs continuously during alarm
  if (runKWSInference()) {
    Serial.println("[KWS] Command detected during alarm");
    sluCallerState = STATE_ALARM_RINGING;
    enterState(STATE_AWAIT_SLU);
  }
}

void handleAwaitSLU() {
  // Timeout — ring buffer contents are stale if we wait too long
  if (millis() - stateEnteredAt > SLU_TIMEOUT_MS) {
    Serial.println("[SLU] Timeout");
    enterState(sluCallerState == STATE_ALARM_RINGING
                 ? STATE_ALARM_RINGING
                 : STATE_IDLE);
    return;
  }

  // SLU reads ring buffer, runs MFE + TFLite, returns result
  int   intentIdx   = -1;
  float intentScore = 0.0f;

  if (runSLUInference(&intentIdx, &intentScore)) {
    lastIntent      = intentIdx;
    lastIntentScore = intentScore;
    dispatchIntent(intentIdx, intentScore);
  }
}

void handleAwaitLid() {
  if (millis() - stateEnteredAt > LID_TIMEOUT_MS) {
    Serial.println("[SENSOR] Lid timeout");
    showOnOLED("Lid not opened", "Reminding later");
    enterState(STATE_REMIND_LATER);
    return;
  }
  if (readHallSensor()) {
    Serial.println("[SENSOR] Lid open");
    enterState(STATE_AWAIT_HAND);
  }
}

void handleAwaitHand() {
  if (millis() - stateEnteredAt > HAND_TIMEOUT_MS) {
    Serial.println("[SENSOR] Hand timeout");
    showOnOLED("Hand not detected", "Reminding later");
    enterState(STATE_REMIND_LATER);
    return;
  }
  if (readHandPresent()) {
    Serial.println("[SENSOR] Hand detected — confirmed");
    enterState(STATE_EVENT_CONFIRMED);
  }
}

void handleEventConfirmed() {
  DateTime now = rtc.now();
  logToSD("confirm_taken", lastIntentScore, now);
  sendEmail("Medicine Taken",
            "Patient confirmed and physically took their medicine.");
  postToDashboard("confirm_taken", lastIntentScore);
  triggerConfirmTaken();
  delay(3000);
  enterState(STATE_IDLE);
}

void handleRemindLater() {
  if (millis() - remindLaterAt >= REMIND_DELAY_MS) {
    Serial.println("[REMIND] 5 min elapsed — re-alarming");
    enterState(STATE_ALARM_RINGING);
  }
}

void handleSOS() {
  DateTime now = rtc.now();
  sendEmail("!! SOS ALERT !!",
            "Patient triggered SOS. Please check immediately.");
  postToDashboard("notify_sos", 1.0f);
  logToSD("notify_sos", 1.0f, now);
  triggerNotifySOS();
  delay(3000);
  enterState(STATE_IDLE);
}

// ============================================================
// TRIGGER STUBS
// TODO: move to triggers.h once TTS / LED patterns are added
// ============================================================

void triggerConfirmTaken() {
  Serial.println("[TRIGGER] confirm_taken");
  showOnOLED("Well done!", "Dose logged");
  // TODO: TTS "Great, your medicine has been confirmed and logged."
  // TODO: green LED blink pattern
}

void triggerDenyTaken() {
  Serial.println("[TRIGGER] deny_taken");
  showOnOLED("Okay!", "Reminding later");
  // TODO: TTS "Okay, I'll remind you again in 5 minutes."
}

void triggerRemindLater() {
  Serial.println("[TRIGGER] remind_later");
  showOnOLED("No problem!", "Remind in 5 min");
  // TODO: TTS "Sure, reminding you again shortly."
}

void triggerAskTime() {
  Serial.println("[TRIGGER] ask_time");
  DateTime now = rtc.now();
  char buf[24];
  snprintf(buf, sizeof(buf), "%02d:%02d", now.hour(), now.minute());
  showOnOLED("Current time:", buf);
  Serial.printf("[TRIGGER] Time: %s\n", buf);
  // TODO: TTS "The time is HH:MM"
}

void triggerAskMedDetails() {
  Serial.println("[TRIGGER] ask_med_details");
  showOnOLED("Medicine:", "Metformin 500mg");
  // TODO: pull from config / SD card
  // TODO: TTS "Your medicine is Metformin 500mg, one tablet"
}

void triggerAskSchedule() {
  Serial.println("[TRIGGER] ask_schedule");
  showOnOLED("Schedule:", "8am / 8pm");
  // TODO: pull from config
  // TODO: TTS "Your schedule is 8am and 8pm daily"
}

void triggerNotifySOS() {
  Serial.println("[TRIGGER] notify_sos");
  showOnOLED("!! SOS SENT !!", "Help coming");
  buzz(1000); delay(200); buzz(1000);
  // TODO: TTS "Emergency alert sent to your caregiver."
  // TODO: GSM SMS
}

void triggerIrrelevant() {
  Serial.println("[TRIGGER] irrelevant — no action");
  // TODO: TTS "Sorry, I didn't understand that."
}

// ============================================================
// SD LOGGING
// ============================================================

void logToSD(const char* label, float confidence, DateTime timestamp) {
  File file = SD.open("/log.csv", FILE_APPEND);
  if (!file) {
    Serial.println("[SD] Cannot open log.csv");
    return;
  }
  float distance = readDistanceCM();
  bool  hall     = readHallSensor();
  char  line[160];
  snprintf(line, sizeof(line),
    "%04d-%02d-%02d %02d:%02d:%02d,%s,%.3f,%.1f,%d",
    timestamp.year(), timestamp.month(), timestamp.day(),
    timestamp.hour(), timestamp.minute(), timestamp.second(),
    label, confidence, distance, hall ? 1 : 0);
  file.println(line);
  file.close();
  Serial.printf("[SD] %s\n", line);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== MedReminder Boot ===");

  // Clear ring buffer
  memset(audio_ring_buf, 0, sizeof(audio_ring_buf));
  audio_ring_head = 0;
  audio_ring_full = false;

  initSensors();    // OLED, buzzer, LED, hall, ultrasonic
  initWiFi();       // WiFi + secure client
  initKWS();        // I2S driver + KWS model — must be first
  initSLU();        // TFLite arena + SLU model

  Wire.begin();
  if (!rtc.begin()) {
    Serial.println("[RTC] Init failed");
  } else if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println("[RTC] Reset to compile time");
  }

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS)) {
    Serial.println("[SD] Init failed");
  } else {
    if (!SD.exists("/log.csv")) {
      File f = SD.open("/log.csv", FILE_WRITE);
      if (f) {
        f.println("timestamp,label,confidence,distance_cm,hall");
        f.close();
      }
    }
    Serial.println("[SD] Ready");
  }

  Serial.println("=== Boot complete ===");
  Serial.println("Send 'A' over Serial to trigger alarm (MVP)");
  enterState(STATE_IDLE);
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  switch (currentState) {
    case STATE_IDLE:            handleIdle();            break;
    case STATE_ALARM_RINGING:   handleAlarmRinging();    break;
    case STATE_AWAIT_SLU:       handleAwaitSLU();        break;
    case STATE_AWAIT_LID:       handleAwaitLid();        break;
    case STATE_AWAIT_HAND:      handleAwaitHand();       break;
    case STATE_EVENT_CONFIRMED: handleEventConfirmed();  break;
    case STATE_REMIND_LATER:    handleRemindLater();     break;
    case STATE_SOS:             handleSOS();             break;
  }
}
