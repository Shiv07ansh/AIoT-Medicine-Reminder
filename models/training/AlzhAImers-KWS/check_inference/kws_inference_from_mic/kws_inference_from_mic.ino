#include <Arduino.h>
#include "driver/i2s.h"

/* KWS Model */
#include <KWS-alzhAImers_inferencing.h>
#include "C:\Users\Project-2501\Documents\Arduino\libraries\Kws_AI_f32\model-parameters\model_metadata.h"
#include <model-parameters\model_metadata.h>
#include "C:\Users\Project-2501\Documents\Arduino\libraries\Kws_AI_f32\edge-impulse-sdk\classifier\ei_run_classifier.h"
#include <edge-impulse-sdk\classifier\ei_run_classifier.h>

#define I2S_WS         42   // LRCLK
#define I2S_SD         41   // Data in from INMP441
#define I2S_SCK        40   // Bit clock

#define SAMPLE_RATE        16000
#define BUFFER_SIZE        16000  // 1 second
#define STRIDE_SAMPLES     8000   // 0.5 seconds stride
#define CONFIDENCE_THRESH  0.85f  // Class must cross this to trigger

int16_t i2s_buffer[BUFFER_SIZE * 2]; // Double buffer for rolling input
size_t write_index = 0;

int16_t audio_window[BUFFER_SIZE];  // 1s chunk for classification
unsigned long last_inference_time = 0;

void setup_i2s() {
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        .use_apll = false,
    };

    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_in_num = I2S_SD,
        .data_out_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
}


bool read_sample(int16_t* sample) {
    size_t bytes_read;
    int32_t temp;
    esp_err_t res = i2s_read(I2S_NUM_0, &temp, sizeof(temp), &bytes_read, portMAX_DELAY);
    if (res == ESP_OK && bytes_read == 4) {
        *sample = temp >> 14; // Downscale to 16-bit
        return true;
    }
    return false;
}
/*
void read_audio(int16_t* buffer, size_t num_samples) {
    size_t bytes_read = 0;
    i2s_read(I2S_NUM_0, (void*)buffer, num_samples * sizeof(int16_t), &bytes_read, portMAX_DELAY);
}
*/
void run_inference_on_audio(int16_t *audio_data) {
    signal_t signal;
    int ret = numpy::signal_from_audio(audio_data, BUFFER_SIZE, &signal);
    if (ret != 0) {
        ei_printf("Failed to create signal from audio\n");
        return;
    }

    ei_impulse_result_t result = {};
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

    if (res != EI_IMPULSE_OK) {
        ei_printf("Inference failed (%d)\n", res);
        return;
    }

    ei_printf("Predictions:\n");
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        float conf = result.classification[i].value;
        const char* label = result.classification[i].label;

        ei_printf("  %s: %.4f\n", label, conf);

        if (conf >= CONFIDENCE_THRESH) {
            Serial.print(">> Triggered intent: ");
            Serial.println(label);
            // You can call a function here: trigger_action(label);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    setup_i2s();
    ei_printf("System ready. Listening...\n");
}

void loop() {
    int16_t sample;
    if (read_sample(&sample)) {
        i2s_buffer[write_index] = sample;
        write_index = (write_index + 1) % (BUFFER_SIZE * 2);
    }

    if (millis() - last_inference_time >= 500) {
        last_inference_time = millis();

        for (int i = 0; i < BUFFER_SIZE; i++) {
            size_t idx = (write_index + BUFFER_SIZE * 2 - BUFFER_SIZE + i) % (BUFFER_SIZE * 2);
            audio_window[i] = i2s_buffer[idx];
        }

        run_inference_on_audio(audio_window);
    }
}
