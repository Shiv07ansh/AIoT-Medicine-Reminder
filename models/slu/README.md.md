# SLU Model — Training Decisions Log

Trained in Keras on Google Colab. See slu_training.ipynb 
for the full training run.

---

## Goal

Classify short verbal responses (typically 1-5 words) from 
an elderly user into one of 8 intent categories, running 
within the remaining SRAM budget after KWS and firmware 
are loaded.

---

## Key Architecture Decision: Text-Based Input

SLU operates on tokenized text, not continued audio features 
from the KWS pipeline.

Why: SLU accuracy on short intent phrases improves with text 
representation. Audio features carry speaker variation and noise 
that does not affect semantic meaning. Text normalises these away.

Tradeoff: requires a transcription step between KWS trigger 
and SLU input, adding latency. Acceptable given the 5-second 
confirmation window in the FSM.

---

## Dataset

- ~180-190 samples per class, 8 classes
- ~1,440-1,520 total samples
- Same recording setup and augmentation pipeline as KWS
- Training environment: Keras on Google Colab
- Split: 80% train / 10% validation / 10% test
- Single speaker, English only

Augmentation:
[data_augmentation.py](data_augmentation.py.md)

---

## Tokenizer
```python
tokenizer = Tokenizer(num_words=1500, oov_token="<OOV>")
sequences = pad_sequences(sequences, maxlen=15, padding='post')
```

Vocab size 1500: tested 500, 1000, 1500, 2000. Beyond 1500 
the embedding layer grew without meaningful accuracy gain 
on this dataset's vocabulary.

Max length 15: accommodates longest realistic user responses. 
Shorter sequences are zero-padded, longer are truncated.

---

## Architecture
```python
model = Sequential([
    Embedding(input_dim=vocab_size, output_dim=150, input_length=15),
    GlobalAveragePooling1D(),
    Dense(94, activation='relu'),
    Dropout(0.3),
    Dense(num_classes, activation='softmax')
])
```

### Embedding Dimension: 150
Tuned empirically: 128 (underfit) → 196 (overfit, too many 
parameters for dataset size) → 150 (best validation accuracy).

### GlobalAveragePooling1D over GRU/LSTM
Initial architecture used bidirectional LSTM. Two problems:

1. TFLite Micro does not include the ReverseV2 kernel.
   Bidirectional layers fail at runtime with op-resolution errors.
   No workaround other than replacing with unidirectional layers.

2. LSTM hidden state adds significant SRAM overhead not available 
   after KWS arena allocation.

GlobalAveragePooling1D deploys cleanly on TFLM, uses minimal 
memory, and the accuracy difference on 1-5 word inputs is small.

### Dense 94 units
Standard dense layer. 94 chosen during hyperparameter search 
alongside embedding_dim tuning.

---

## Training Configuration

- Loss: categorical cross-entropy
- Optimizer: Adam, lr=0.001
- Batch size: 16
- Max epochs: 50
- Early stopping: patience=5 on validation loss
- Convergence: typically 12-20 epochs

---

## Quantization

Post-training int8 via TFLite converter in Colab.

Check which path was used — dynamic range vs full integer:
```python
# Dynamic range (float32 input, int8 weights)
converter.optimizations = [tf.lite.Optimize.DEFAULT]

# Full integer (int8 input and output)
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8
converter.inference_output_type = tf.int8
```

Check input type at runtime to confirm:
```cpp
TfLiteTensor* input = interpreter->input(0);
Serial.println(input->type); // 9=int8, 10=uint8, 1=float32
```

Convert to C array after quantization:
```bash
xxd -i slu_model.tflite > slu_model_data.cc
```

Add const and alignment:
```cpp
alignas(16) const unsigned char slu_model_tflite[] = { ... };
const unsigned int slu_model_tflite_len = 123456;
```

---

## Op Resolver

Use MicroMutableOpResolver. Ops to register:
- FullyConnected
- Reshape
- Softmax
- Mean (for GlobalAveragePooling1D)

Identified via micro_mutable_op_resolver.h runtime error codes.

---

## Tensor Arena

Separate arena from KWS, also allocated in PSRAM:
```cpp
EXT_RAM_BSS_ATTR static uint8_t slu_tensor_arena[SLU_ARENA_SIZE];
```

Same empirical sizing process as KWS.
Final size: approximately 80-100KB.

---

## Results

| Metric | Value |
|--------|-------|
| Accuracy | 94.1% |
| Inference latency | ~70ms @ 240MHz |
| Flash | ~1.1MB |
| SRAM | ~200KB |

Performance on unseen speakers and non-English inputs 
not formally benchmarked.
See [../../../LIMITATIONS_AND_FUTURE_WORK.md](../../../LIMITATIONS_AND_FUTURE_WORK.md)