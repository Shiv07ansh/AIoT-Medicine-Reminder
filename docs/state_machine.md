# System State Machine

The firmware implements a four-state FSM that orchestrates 
all device interactions. This document describes each state, 
transition condition, timeout behaviour, and the intent-to-action 
mapping that drives system outputs.

---

## States
```
┌─────────────────────────────────────────────────────────────┐
│                                                             │
│   IDLE ──── KWS detects trigger ──→ AWAIT_SLU               │
│     ↑                                    │                  │
│     │                              SLU classifies           │
│     │                                    │                  │ 
│     │                              ┌─────┴──────┐           │
│     │                    confirm_taken    all other intents |
│     │                              │          │             │
│     │                         AWAIT_LID   handle            │
│     │                              │    directly            │
│     │              lid_open=true   │                        │
│     │                         AWAIT_HAND                    │
│     │                              │                        │
│     │         hand_present=true    │                        │
│     │                         CONFIRMED                     │
│     │                              │                        │
│     └──────── log + notify ────────┘                        │
│                                                             │
│  All states have 5s timeout → reset to IDLE                 │
└─────────────────────────────────────────────────────────────┘
```

---

## State Descriptions

### IDLE
- KWS model runs continuously on audio stream
- No sensor checks active
- Display shows current time or idle message
- System remains in this state until KWS confidence 
  exceeds calibrated threshold

### AWAIT_SLU
- Entered after KWS wake-word detection
- Captures next ~1s of audio
- Runs SLU inference on captured audio
- Routes to appropriate handler based on classified intent
- 5s timeout → IDLE if no intent classified

### AWAIT_LID
- Entered only on confirm_taken intent
- Monitors Hall effect sensor for lid_open = true
- Display: "Please open your medicine box"
- 5s timeout → IDLE, dose not confirmed

### AWAIT_HAND
- Entered after lid_open confirmed
- Monitors ultrasonic sensor for hand_present = true
  (distance < d_threshold = d_empty - 10mm)
- Display: "Please take your medicine"
- 5s timeout → IDLE, dose not confirmed

### EVENT CONFIRMED
- All three conditions met: intent + lid + hand
- Log dose as taken with timestamp
- Send email/SMS notification to caregiver
- Display: "Medicine taken confirmed"
- TTS: confirmation playback
- Reset to IDLE

---

## Intent-to-Action Mapping

| Intent | State Entered | Action |
|--------|--------------|--------|
| confirm_taken | AWAIT_LID | Begin sensor verification sequence |
| deny_taken | IDLE | Reschedule alarm (nextAlarmTime += interval) |
| remind_later | IDLE | Extract delay, postpone nextAlarmTime |
| ask_med_details | IDLE | Read medicineRegistry, OLED + TTS output |
| ask_schedule | IDLE | Read reminderQueue.peek(), display next dose |
| ask_time | IDLE | RTC lookup, display + TTS |
| notify_sos | IDLE | BYPASS sensor checks, immediate email + SMS |
| irrelevant | IDLE | Confidence below threshold, do nothing |

---

## Alarm State Flags
```cpp
bool alarmDue;        // Timer ISR raises this at nextAlarmTime
bool snoozed;         // remind_later sets this
bool completed;       // EVENT CONFIRMED sets this
bool kwsActive;       // Currently in KWS inference loop
bool sluActive;       // Currently in SLU inference
bool lidOpened;       // Hall sensor ISR sets this
bool handDetected;    // Ultrasonic ISR sets this
```

---

## Alarm Escalation

If reminder goes unacknowledged:
- After 30s: reschedule nextAlarmTime += 5 minutes
- Up to 3 retries
- After 3 retries: send missed-dose notification to caregiver
- Only one active reminder at a time (completed/snoozed flags 
  prevent overlap)

---

## SOS Handling

SOS bypasses the normal three-factor confirmation entirely. 
On notify_sos classification:
1. Require two successive SOS detections within 2s 
   (prevents accidental trigger on casual use of "help")
2. Format emergency email/SMS with timestamp, device ID
3. Activate local audible alarm (buzzer)
4. TTS: "Emergency alert sent"
5. Hardware watchdog monitors network — if send fails 
   repeatedly, escalate locally
