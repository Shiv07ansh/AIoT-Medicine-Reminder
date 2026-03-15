# Frequently Asked Questions

Quick answers and pointers to detailed documentation.

---

## About the System

**What does this system actually do?**  
A voice-interactive medicine reminder that runs entirely offline 
on a $12 microcontroller. It alarms, listens for the user's 
verbal response, classifies intent, and physically verifies 
that the medicine box was opened and accessed before confirming 
adherence. Full walkthrough: [interaction_flow.md](interaction_flow.md)

**Why does it need three forms of confirmation: voice, lid, 
and hand?**  
Voice confirmation alone is unreliable for cognitively impaired 
users who may say "yes" without actually taking medicine. 
Physical sensor cross-validation ensures the box was opened 
and accessed, not just verbally acknowledged. A single-signal 
system can be fooled by accident or confusion. The three-factor 
approach is the core reliability argument of the design. 
Full details in [docs/interaction_flow.md](docs/interaction_flow.md) , [docs/state_machine.md](docs/state_machine.md)

**Why does it work offline?**  
The target users such as elderly individuals in rural or low-income 
settings cannot be assumed to have stable internet. Cloud 
dependency would make the system fail precisely when and where 
it's needed most. All inference runs locally on ESP32-S3. 
WiFi is used only for optional caregiver email notifications.

**What happens if the user doesn't respond?**  
The alarm escalates, retried up to 3 times at 5-minute 
intervals. After 3 unacknowledged reminders, a missed-dose 
notification is sent to the caregiver. 
[docs/interaction_flow.md](docs/interaction_flow.md) — Alarm Escalation

**What if there's no WiFi?**  
Email notifications are disabled. All other functionality 
works normally alarms, voice interaction, sensor 
verification, and local logging to CSV on SD card. 

**What are the alerting fallbacks?**  
Primary: email via SMTP over WiFi  
Secondary: SMS via GSM module (SIM800L + TinyGSM) — 
works without WiFi  
Tertiary: local audible alarm (buzzer) — works without 
any connectivity  
Full details in [docs/interaction_flow.md](../docs/interaction_flow.md) — Emergency SOS flow

---

## About the Models

**Why two models instead of one?**  
A single end-to-end model doing both keyword detection and 
intent classification simultaneously exceeded the ESP32-S3's 
SRAM budget and introduced unacceptable latency. The two-stage 
cascade keeps KWS lightweight and always-on (~30ms, 130KB) 
while SLU only activates on trigger (~75ms, 200KB). 
Cascading inference architecture — the always-on KWS module triggers the SLU module only on keyword detection, minimising unnecessary compute cycles and power consumption.
Full details in [models/README.md](../models/README.md)

**Why does KWS use audio features but SLU uses text?**  
KWS is a binary audio detection task — MFE features work well 
and are computationally efficient. SLU is a semantic 
classification task, text representation normalises away 
speaker variation and noise that doesn't affect meaning. 
Different problems, different representations.  
Full details in: [models/README.md](models/README.md)— Note on Architecture Divergence

**Why MFE instead of MFCC for KWS?**  
MFE avoids the DCT stage, making it computationally lighter 
with no meaningful accuracy difference for a binary detection 
task. More efficient for real-time inference on a 
resource-constrained MCU.
Full details in: [models/kws/training/README.md](models/kws/training/README.md) — Feature Extraction

**Why int8 quantization?**  
float32 models exceed SRAM budget. uint8 showed more accuracy 
degradation than int8 on this dataset. int8 post-training 
quantization reduced model size by ~75% with less than 2% 
accuracy drop.  
Full details in: [models/kws/training/README.md](models/kws/training/README.md) — Quantization Decision

**How accurate are the models?**  
KWS: 97.2% | SLU: 95% — both on controlled single-speaker 
test set. Performance on unseen speakers degrades and has 
not been formally benchmarked. This is the core limitation.  
Full details in: [models/kws/model_card.md](models/kws/model_card.md), [models/slu/model_card.md](models/slu/model_card.md) , 
[LIMITATIONS_AND_FUTURE_WORK.md](LIMITATIONS_AND_FUTURE_WORK.md)

**Why does accuracy degrade on other people's voices?**  
Both models were trained primarily on a single speaker. 
The dataset lacked linguistic and acoustic diversity. 
Generalisation to unseen speakers, accents, and languages 
is the primary open research problem this work surfaces.  
Full details in: [LIMITATIONS_AND_FUTURE_WORK.md](LIMITATIONS_AND_FUTURE_WORK.md)

---

## About the Hardware

**Why ESP32-S3 specifically?**  
Vector extensions accelerate fixed-point convolution for 
quantized inference. External PSRAM support allows tensor 
arena allocation without exhausting internal SRAM. 
Deep-sleep sub-5µA enables battery operation.  
Full details in: [docs/hardware_setup.md](docs/hardware_setup.md) — MCU Selection Analysis

**Why does the HC-SR04 need a voltage divider?**  
HC-SR04 echo pin outputs 5V. ESP32-S3 GPIOs are 3.3V maximum. 
Direct connection damages the GPIO. The 10kΩ + 20kΩ divider 
brings echo output to 3.33V.  
Full details in: [docs/hardware_setup.md](docs/hardware_setup.md) — Voltage Level Shifting

**How was the ultrasonic threshold calibrated?**  
d_threshold = d_empty - 10mm  
Baseline d_empty measured when box is empty. Any distance 
reading 10mm below that baseline indicates hand insertion. 
Median filter over 5 readings prevents false positives.  
Full details in: [docs/hardware_setup.md](docs/hardware_setup.md) — Sensor Placement and Tuning

**Why is the hall effect sensor output sometimes unreliable?**  
Sensitive to magnet alignment — must be within 5mm tolerance 
of A3144 sensor face. Solved by epoxying components in place 
after positioning with a hardware jig.  
[docs/hardware_setup.md](docs/hardware_setup.md) — Sensor Alignment Sensitivity

**How much does the hardware cost?**  
Approximately $12.40 total BOM.  
Full details in: [BOM.md](BOM.md) — Bill of Materials

---

## About the Firmware

**How is the system flow controlled?**  
A four-state finite state machine: IDLE → AWAIT_SLU → 
AWAIT_LID → AWAIT_HAND → CONFIRMED. Each state has a 5s 
timeout that resets to IDLE.  
Full details: [docs/state_machine.md](docs/state_machine.md)

**Why is the tensor arena in PSRAM and not internal SRAM?**  
Both KWS and SLU arenas together with firmware exceed internal 
SRAM. PSRAM allocation via EXT_RAM_BSS_ATTR frees internal 
SRAM for the TFLM runtime stack.  
Full details in: [firmware/README.md](firmware/README.md) — Tensor Arena

**How were the op resolver operators determined?**  
By reading the error codes thrown by micro_mutable_op_resolver.h 
at runtime — each missing op throws a specific error identifying 
exactly which op needs to be registered.  
Full details in: [firmware/README.md](firmware/README.md) — Op Resolver

---

## About the Research

**Where is the paper?**  
Accepted, ICDECT-2025 Conference Springer LNNS. 
Zenodo preprint: https://zenodo.org/records/19034554  
DOI: 10.5281/zenodo.19034554  
Full details in: [docs/paper.md](docs/paper.md)

**What are the open research questions?**  
Primarily: generalising KWS/SLU to unseen speakers within 
MCU memory constraints, and neural architecture search for 
maximising accuracy within fixed SRAM budgets.  
Full details in: [LIMITATIONS_AND_FUTURE_WORK.md](LIMITATIONS_AND_FUTURE_WORK.md)

**What is this repo relative to the paper?**  
The paper documents the research contribution. This repo 
contains the complete implementation including engineering 
decisions, failure modes, hardware challenges, and training 
decisions that could not fit in the paper due to space 
constraints. The repo is the extended artifact.
