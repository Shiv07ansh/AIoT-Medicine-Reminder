# KWS Model — Training Decisions Log

Chronological record of decisions made during KWS training.
Trained entirely in Edge Impulse — no custom training code.

---

## Goal

Train a keyword spotting model that:
- Runs continuously on ESP32-S3 within SRAM budget
- Detects medicine-related commands vs background noise
- Completes inference in under 40ms
- Fits within ~130KB SRAM and ~0.7MB flash

---

## Dataset

Custom recorded trigger phrases at 16kHz in .wav format.

- ~2,500 total samples
- 1,500 positive (all medicine-related command utterances, 
  collapsed to single binary label — intent distinction 
  is irrelevant at this detection stage)
- 1,500 negative (ambient noise, silence, ESC-50 clips, 
  Google Speech Commands background samples)
- Split: 80% train / 20% test
- Single speaker throughout

Augmentation pipeline shared with SLU:
[data_augmentation.py](data_augmentation.py.md)

Techniques: additive noise (5dB/10dB SNR), time stretch 
(0.9x/1.1x), pitch shift (+-1 semitone). 6x total expansion.

---

## Feature Extraction

MFE (Mel Frequency Energy) chosen over MFCC:
- Avoids the DCT stage, computationally lighter
- No meaningful accuracy difference for binary detection
- Edge Impulse default for keyword spotting tasks

Parameters:
- Sampling rate: 16kHz
- Frame window: 30ms (480 samples)
- Frame stride: 10ms (160 samples)
- Mel filterbanks: 40
- Raw output: 98x40
- Model input: 49x40 (pairs of frames averaged, Edge Impulse default)

Pre-emphasis: applied by Edge Impulse DSP pipeline (coefficient 0.98).
Frequency range: confirm from Edge Impulse project DSP settings.

---

## Architecture

2D CNN treating the MFE feature matrix as an image.
```
Input: [49, 40] MFE features
Conv2D (32 filters, 3x3, ReLU)   → (96, 38, 32)
MaxPooling2D (2x2)                → (48, 19, 32)
Conv2D (64 filters, 3x3, ReLU)   → (46, 17, 64)
MaxPooling2D (2x2)                → (23, 8, 64)
Flatten                           → (11776)
Dense (64, ReLU)                  → (64)
Dense + Softmax                   → (7)
```

Note: paper described 1D depthwise separable CNN but the actual 
exported model summary confirmed 2D Conv architecture above. 
The model summary is the ground truth.

---

## Training Configuration

- Trained in Edge Impulse Studio
- CMSIS-NN optimisations enabled for ARM Cortex-M targets
- Loss: categorical cross-entropy
- Optimizer: Adam, lr=0.003
- Early stopping on validation accuracy

---

## Quantization

Post-training int8 quantization via Edge Impulse export pipeline.

- float32 ruled out immediately: model exceeded SRAM budget
- uint8 tested: more accuracy degradation than int8
- int8 settled on: <2% accuracy drop, ~75% size reduction vs float32

Check input tensor type at runtime before feeding data:
```cpp
TfLiteTensor* input = interpreter->input(0);
Serial.println(input->type); // 9=int8, 10=uint8, 1=float32
```

---

## Op Resolver

Use MicroMutableOpResolver, not AllOpsResolver.
AllOpsResolver loads backpropagation ops unnecessary for inference
and caused SRAM overflow.

Ops to register (identified via micro_mutable_op_resolver.h 
error codes at runtime, not from Netron):
- DepthwiseConv2D
- Conv2D
- MaxPool2D
- Reshape
- Softmax
- ReLU
- FullyConnected

---

## Tensor Arena

Allocated in PSRAM to avoid internal SRAM exhaustion:
```cpp
EXT_RAM_BSS_ATTR static uint8_t kws_tensor_arena[KWS_ARENA_SIZE];
```

Size determined empirically:
1. Start at 200KB
2. Run inference on device
3. If crash: increase by 8KB and retry
4. If stable: decrease by 1KB until crash
5. Add 2KB margin
6. Final size: approximately 60-80KB

---

## Model Conversion

After Edge Impulse export, convert .tflite to C array:
```bash
# Linux/Mac
xxd -i kws_model.tflite > kws_model_data.cc

# Windows
certutil -encodehex kws_model.tflite kws_model_data.cc 4
```

Add const and alignment to the generated array:
```cpp
alignas(16) const unsigned char kws_model_tflite[] = { ... };
const unsigned int kws_model_tflite_len = 764321;
```

Without const the array is copied into SRAM at runtime.
With const it stays in Flash. Always use const.

---

## Results

| Metric | Value |
|--------|-------|
| Accuracy | 96.4% |
| Inference latency | ~30ms @ 240MHz |
| Flash | ~0.7MB |
| SRAM | ~130KB |
| Quantization accuracy drop | <2% |

Performance on unseen speakers degrades — not formally benchmarked.
See [../../LIMITATIONS_AND_FUTURE_WORK.md](../../LIMITATIONS_AND_FUTURE_WORK.md)
