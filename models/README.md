# Models

Two models run sequentially on-device to process the user's 
verbal response to a medication prompt.

---

## Pipeline
```
Continuous audio → KWS detects trigger → SLU classifies intent
                                              │
                              ┌───────────────┴─────────┐
                         Voice intent               Sensor checks
                         (SLU output)          (Hall + Ultrasonic)
                              │                         │
                              └──────── State machine ──┘
                                              │
                                        Action triggered
```

---

## Why Two Sequential Models

A single end-to-end model doing both KWS and SLU simultaneously 
exceeded the SRAM budget and introduced unacceptable inference 
latency. The two-stage cascade keeps each model small and 
purpose-specific:

- KWS runs continuously at low cost (~30ms, 130KB SRAM)
- SLU activates only on KWS trigger (~75ms, 200KB SRAM)

This mirrors the architecture of commercial voice assistants 
(Alexa, Siri) — lightweight always-on detector gates a 
heavier intent classifier.

---

## KWS — Keyword Spotting

**Task:** Binary classification — medicine-related command vs noise  
**Architecture:** 1D depthwise separable CNN  
**Input:** 49×40 MFE feature matrix  
**Output:** keyword / noise probability  
**Accuracy:** 97.2%  
**Latency:** ~30ms  
→ Full details: kws/model_card.md

## SLU — Spoken Language Understanding

**Task:** 8-class intent classification  
**Architecture:** Embedding + GlobalAveragePooling + Dense  
**Input:** tokenized + padded text sequence (vocab=1500, max_len=15)  
**Output:** probability over 8 intent categories  
**Accuracy:** 95%  
**Latency:** ~75ms  
→ Full details: slu/model_card.md

---

## Shared Constraints

Both models operate under the same hard constraints:
- Combined must fit within ESP32-S3 SRAM alongside firmware
- Inference must complete in real-time (total <150ms)
- int8 quantization applied to both
- No backpropagation ops — inference only
- Tensor arena allocated in PSRAM, not internal SRAM

---

## Note on Architecture Divergence

KWS operates on raw audio features (MFE — signal processing 
pipeline). SLU operates on tokenized text (NLP pipeline). 
This is a deliberate architectural decision: SLU accuracy 
on short intent phrases improves significantly when operating 
on the text representation rather than continuing the audio 
feature pipeline from KWS. The tradeoff is requiring a 
text transcription step between KWS trigger and SLU input.
text transcription step between KWS trigger and SLU input.