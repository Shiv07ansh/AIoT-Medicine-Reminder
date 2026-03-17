## Plugging In Models — Checklist

### KWS (Edge Impulse)
- [ ] Export project as Arduino library, install in IDE
- [ ] Uncomment `#include <KWS-alzhAImers_inferencing.h>` in `kws.h`
- [ ] Replace `kws_flat[16000]` with `kws_flat[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]`
- [ ] Update `KWS_POSITIVE_LABEL` to match your Edge Impulse label name
- [ ] Uncomment `signal_t` → `run_classifier()` → result block
- [ ] Remove `return false` MVP stub
- [ ] Tune `KWS_CONFIDENCE_THR`

### SLU (TFLite Micro)
- [ ] Place `slu_model_int8.h` in sketch folder
- [ ] Uncomment `#include "slu_model_int8.h"` in `slu.h`
- [ ] Uncomment model load + `AllocateTensors()` block in `initSLU()`
- [ ] Boot once with tensor print block uncommented — read scale/zp from Serial
- [ ] Fill `SLU_INP_SCALE`, `SLU_INP_ZERO_POINT`, `SLU_OUT_SCALE`, `SLU_OUT_ZERO_POINT`
- [ ] Verify op list against model using Netron (https://netron.app)
- [ ] Remove MVP stub in `runSLUInference()`
- [ ] Tune `SLU_CONFIDENCE_THR`

### Credentials
- [ ] Fill `credentials.h` with WiFi SSID/password
- [ ] Fill SMTP server, port, from/to addresses
- [ ] Base64-encode email credentials into `EMAIL_USER_B64` / `EMAIL_PASS_B64`
- [ ] Add `credentials.h` to `.gitignore`

---

## TODO Summary

| Location | What |
|----------|------|
| `kws.h` | Uncomment Edge Impulse inference block |
| `slu.h` | Uncomment TFLite model load + inference |
| `slu.h` | Fill quantization params after first boot |
| `sensors.h` | Calibrate `D_EMPTY_MM` for your specific box |
| All triggers | Implement TTS audio playback |
| `triggerNotifySOS()` | Add GSM SMS via modem |
| `handleAwaitSLU()` | Consider adding SLU timeout back to ALARM_RINGING with re-prompt |
| `MedReminder.ino` | Pull medicine name/schedule from SD config file |
| `connectivity.h` | Replace `setInsecure()` with CA cert for production |
