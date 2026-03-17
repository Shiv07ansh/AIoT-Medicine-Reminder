# KWS Model Card — Keyword Spotting

## Summary

Binary classifier that runs continuously on-device to detect 
medicine-related commands in the audio stream. Gates the SLU 
model — SLU only activates on a positive KWS detection.

---

## Architecture

2D CNN operating on MFE spectrogram features.

| Layer | Output Shape | Params |
|-------|-------------|--------|
| Conv2D (32 filters, 3x3, ReLU) | (96, 38, 32) | 320 |
| MaxPooling2D (2x2) | (48, 19, 32) | 0 |
| Conv2D (64 filters, 3x3, ReLU) | (46, 17, 64) | 18,496 |
| MaxPooling2D (2x2) | (23, 8, 64) | 0 |
| Flatten | (11776) | 0 |
| Dense (64, ReLU) | (64) | 753,728 |
| Dense + Softmax | (7) | 455 |

**Total parameters:** ~773,000  
**Trained in:** Edge Impulse  
**Quantization:** int8 post-training, <2% accuracy drop vs float32

---

## Input

| Parameter | Value |
|-----------|-------|
| Sampling rate | 16 kHz |
| Frame window | 30ms (480 samples) |
| Frame stride | 10ms (160 samples) |
| Mel filterbanks | 40 |
| Raw feature shape | 98x40 |
| Model input shape | 49x40 (pairs averaged) |
| Quantization | int8 |

---

## Performance

| Metric | Value |
|--------|-------|
| Accuracy | 96.4% |
| Inference latency | ~30ms @ 240MHz |
| Flash (program storage) | ~0.7MB |
| SRAM (global variables) | ~130KB |
| Quantization accuracy drop | <2% |

---

## Training Data

- ~2,500 total samples
- 1,500 positive (medicine-related command utterances)
- 1,500 negative (ambient noise, silence, ESC-50, Google Speech Commands)
- Augmentation: additive noise (5dB/10dB SNR), time stretch (0.9x/1.1x), pitch shift (+-1 semitone)
- Split: 80% train / 20% test
- Single speaker — primary limitation

---

## Known Limitations

Performance on unseen speakers has not been formally benchmarked 
but degrades noticeably in informal testing. The model was trained 
on a single speaker's voice. Generalisation to regional accents 
and non-native pronunciations is the core unresolved problem.

Full details: [../../LIMITATIONS_AND_FUTURE_WORK.md](../../LIMITATIONS_AND_FUTURE_WORK.md)
