# SLU Model Card — Spoken Language Understanding

## Summary

8-class intent classifier that activates after a KWS trigger. 
Classifies the user's verbal response into one of 8 intent 
categories and routes the system to the appropriate action.

Operates on tokenized text, not raw audio. This is a deliberate 
architectural choice — text representation normalises away speaker 
variation and noise that does not affect semantic meaning.

---

## Architecture

Embedding + GlobalAveragePooling + Dense classifier.

| Layer                  | Output Shape | Notes                          |
| ---------------------- | ------------ | ------------------------------ |
| Embedding              | (15, 150)    | vocab=1500, input_length=15    |
| GlobalAveragePooling1D | (150)        | lightweight alternative to GRU |
| Dense (94, ReLU)       | (94)         |                                |
| Dropout (0.3)          | (94)         |                                |
| Dense + Softmax        | (8)          | intent probabilities           |

**Trained in:** Keras on Google Colab  
**Quantization:** int8 post-training

---

## Input

| Parameter | Value |
|-----------|-------|
| Input type | Tokenized text sequence |
| Vocabulary size | 1,500 |
| Max sequence length | 15 tokens |
| OOV token | `<OOV>` |
| Padding | Post-padding with zeros |

---

## Intent Classes

| Class | Example Triggers | System Action |
|-------|-----------------|---------------|
| confirm_taken | "yes", "I took it", "good" | Enter sensor verification |
| deny_taken | "no", "I didn't", "forgot" | Reschedule alarm |
| remind_later | "snooze", "remind me in 30 min" | Postpone by extracted delay |
| ask_med_details | "which medicine do I take?" | Read registry via TTS |
| ask_schedule | "when is my next dose?" | Display next dose time |
| ask_time | "what time is it?" | RTC lookup + TTS |
| notify_sos | "help", "I need a doctor" | Bypass sensors, trigger alert |
| irrelevant | out-of-scope queries | Confidence below threshold, do nothing |

**Note on irrelevant class:** This class only handles utterances 
that passed KWS but were too ambiguous to classify into a meaningful 
intent. Truly irrelevant speech never reaches SLU — it is filtered 
upstream by KWS. The irrelevant class is a confidence threshold 
mechanism, not a catch-all.

---

## Performance

| Metric | Value |
|--------|-------|
| Accuracy | 94.1% |
| Inference latency | ~70ms @ 240MHz |
| Flash (program storage) | ~1.1MB |
| SRAM (global variables) | ~200KB |

---

## Training Data

- ~180-190 samples per class
- ~1,440-1,520 total samples across 8 classes
- Same augmentation pipeline as KWS
- Split: 80% train / 10% validation / 10% test
- Single speaker, English only — primary limitation

---

## Known Limitations

Single language and accent. Performance on regional accents, 
non-native pronunciations, and non-English inputs is untested. 
The tokenizer vocabulary was built from a single speaker's 
recordings and may not generalise to different speech patterns.

Full details: [../../LIMITATIONS_AND_FUTURE_WORK.md](../../LIMITATIONS_AND_FUTURE_WORK.md)