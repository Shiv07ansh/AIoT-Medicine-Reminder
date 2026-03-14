/* 
  Comprehensive ESP32 Sketch Scaffold:
  - I2S mic input → Edge Impulse KWS inference
  - Peripheral I/O: ultrasonic, hall-effect, buzzer, OLED, I2S speaker, RTC, SD card
  - Connectivity: WiFi (and/or GSM) for dashboard HTTP POST and SMTP email
  - Debug logs and performance timing over Serial
  - Data logging to SD and dashboard
*/


/* ============== HEADERS AND LIBRARIES ================ */

#include <Arduino.h>


/* KWS Model */
#include <KWS-alzhAImers_inferencing.h>
#include "C:\Users\Project-2501\Documents\Arduino\libraries\Kws_AI_f32\model-parameters\model_metadata.h"
#include <model-parameters\model_metadata.h>
#include "C:\Users\Project-2501\Documents\Arduino\libraries\Kws_AI_f32\edge-impulse-sdk\classifier\ei_run_classifier.h"
#include <edge-impulse-sdk\classifier\ei_run_classifier.h>

/* I2S Microphone */
#include <driver/i2s.h>

/* OLED Display */
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

/* RTC Module (ZS-042 / DS3231) */
#include "DS3231.h"

/*SD Card */
#include <SPI.h>
#include <SD.h>

/* Ultrasonic Sensor (HC-SR04) */
    /* No library, just need to initialize pins */

/*GSIM module */


/* WiFi & HTTP & SMTP */
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
/* #include <ArduinoJson.h>  // optional for JSON serialization */

/* ============== HEADERS AND LIBRARIES ================ */

/*--------------------------------------------------------------------------------------------------------------------------------*/

/* ============== CONFIGURATION SECTION  ================ */


/* --- WIFI --- */
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";

/* --- SMTP Email --- */
const char* SMTP_SERVER = "smtp.example.com";
const int   SMTP_PORT = 465; // or 587
const char* EMAIL_USER = "user@example.com";
const char* EMAIL_PASS = "your_email_password";
const char* EMAIL_TO   = "caregiver@example.com";

/* --- GSM --- */

/* --- Dashboard endpoint --- */
const char* DASHBOARD_URL = "https://your-server.example.com/api/log";


/* ------------------- PINS ----------------------------------------------------------------------------------------------------- */

/* I2C for OLED + RTC + VL53L0X + PCF8574 */
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

/* I2S Mic */
#define I2S_BCK      27
#define I2S_WS       33
#define I2S_DATA_IN  32

/* Buzzer */
#define PIN_BUZZER   15

/* SD Card */
#define SD_MOSI      23
#define SD_MISO      19
#define SD_SCK       18
#define SD_CS        5

/* I2C expander (PCF8574) address */ 
#define PCF8574_ADDR 0x20

/* PCF8574 pin assignments (example) */
/* P0: Hall sensor input */
/* P1: spare button input */
/* P2: status LED output */
/* Adjust as needed. */
#define EXP_PIN_HALL   P0
#define EXP_PIN_BTN    P1
#define EXP_PIN_LED    P2

/* === Globals & Objects === */
/* Edge Impulse audio buffer */
static float audio_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

/* Performance monitor */
unsigned long last_inference_ms = 0;
unsigned long inference_time_us = 0;

/* I2C devices */
Adafruit_SSD1306 display(128, 64, &Wire);
RTC_DS3231 rtc;
VL53L0X vl53;
PCF8574 expander(PCF8574_ADDR);

/* WiFi client for HTTP/SMTP */
WiFiClientSecure wifiClientSecure;

/* ============== CONFIGURATION SECTION  ================ */

/*--------------------------------------------------------------------------------------------------------------------------------------*/

/* ============== FUNCTION PROTOTYPES  ================ */

bool initI2SMic();
bool readI2SAudio(float* out_buf, size_t buf_len);
void handleIntent(const char* label, float confidence);
void sendEmail(const char* subject, const char* body);
void postToDashboard(const char* label, float confidence);
void logToSD(const char* label, float confidence, DateTime timestamp);
float readDistanceCM();
bool readHallViaExpander();
void buzz(int ms);
void showOnOLED(const char* line1, const char* line2 = nullptr);
void initPeripherals();
void initWiFi();
void initRTC();
void initSD();
void initI2SMicWrapper();
void runKWSInference();

/* ============== FUNCTION PROTOTYPES  ================ */

/*----------------------------------------------------------------------------------------------------------------------------------------*/


/* =================== VOID SETUP ======================= */
void setup() {
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Medicine Reminder KWS Starting ===");

  /* Initialize peripherals */
  initPeripherals();

  /* Initialize WiFi */
  initWiFi();

  /* Initialize RTC */
  initRTC();

  /* Initialize SD card */
  initSD();

  ei_printf("Edge Impulse classifier ready\n");

/* =================== VOID SETUP ======================= */

/*----------------------------------------------------------------------------------------------------------------------------------------*/


/* =================== VOID LOOP  ======================= */
void loop() {

runKWSInference();
/* Adjust delay for deired inference rate */  
delay(200);

}
/* =================== VOID LOOP  ======================= */

/*----------------------------------------------------------------------------------------------------------------------------------------*/

/*======== Peripheral Initialization ===========*/

void initPeripherals() {
  // I2C bus
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  /* OLED */ 
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("KWS Starting...");
    display.display();
  }

  /* RTC */
  if (!rtc.begin()) {
    Serial.println("RTC init failed");
  } else {
    if (rtc.lostPower()) {
      Serial.println("RTC lost power, setting to compile time");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

  // VL53L0X distance sensor
  if (!vl53.init()) {
    Serial.println("VL53L0X init failed");
  } else {
    vl53.setTimeout(500);
  }

  /* PCF8574 expander */ 
  if (!expander.begin()) {
    Serial.println("PCF8574 init failed");
  } else {
    // Configure expander pins
    expander.pinMode(EXP_PIN_HALL, INPUT);
    expander.pinMode(EXP_PIN_BTN, INPUT);
    expander.pinMode(EXP_PIN_LED, OUTPUT);
    expander.digitalWrite(EXP_PIN_LED, LOW);
  }

  /*Buzzer*/ 
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  /*I2S mic*/ 
  initI2SMicWrapper();

  /* SD: SPI.begin will be in initSD() */
}
/*======== Peripheral Initialization ===========*/

/* === WiFi Initialization === */
void initWiFi() {
  Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start > 10000) {
      Serial.println("\nWiFi connect timeout");
      return;
    }
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  wifiClientSecure.setInsecure(); // for testing; in production use setCACert()
}
/* === WiFi Initialization === */

/* === RTC Initialization (done in initPeripherals) === */
void initRTC() {
  /* already in initPeripherals */
}

/* === SD Initialization === */
void initSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed");
  } else {
    Serial.println("SD initialized");
    // Ensure log file exists
    if (!SD.exists("/log.csv")) {
      File f = SD.open("/log.csv", FILE_WRITE);
      if (f) {
        f.println("timestamp,label,confidence,distance_cm,hall");
        f.close();
      }
    }
  }
}
/* === SD Initialization === */

/* === I2S Mic Setup & Wrapper === */
void initI2SMicWrapper() {
  if (!initI2SMic()) {
    Serial.println("I2S mic init failed");
  } else {
    Serial.println("I2S mic initialized");
  }
}

bool initI2SMic() {
  /*Configure I2S for microphone (PCM)*/ 
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DATA_IN
  };
  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) return false;
  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  return (err == ESP_OK);
}

/* Read I2S audio into float buffer normalized [-1.0, 1.0] */
bool readI2SAudio(float* out_buf, size_t buf_len) {
  /* Placeholder: adapt if your mic returns int16 or float */
  static int16_t i2s_read_buffer[512];
  size_t samples_filled = 0;
  while (samples_filled < buf_len) {
    size_t to_read = min((size_t)512, buf_len - samples_filled);
    size_t bytes_read = 0;
    esp_err_t res = i2s_read(I2S_NUM_0, (void*)i2s_read_buffer, to_read * sizeof(int16_t), &bytes_read, portMAX_DELAY);
    if (res != ESP_OK) {
      Serial.println("I2S read error");
      return false;
    }
    size_t got = bytes_read / sizeof(int16_t);
    for (size_t i = 0; i < got && samples_filled < buf_len; i++) {
      out_buf[samples_filled++] = (float)i2s_read_buffer[i] / 32768.0f;
    }
  }
  return true;
}

/* === I2S Mic Setup & Wrapper === */


/* === KWS Inference === */

void runKWSInference() {
  /* 1. Capture audio into audio_buffer */
  if (!readI2SAudio(audio_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)) {
    Serial.println("Error reading audio");
    delay(100);
    return;
  }

  /* 2. Prepare signal_t */
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  signal.get_data = [](size_t offset, size_t length, float *out_ptr) -> int {
    memcpy(out_ptr, &audio_buffer[offset], length * sizeof(float));
    return 0;
  };

  /* 3. Run inference & time it */
  unsigned long t0 = micros();
  ei_impulse_result_t result;
  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);
  inference_time_us = micros() - t0;
  last_inference_ms = millis();

  if (r != EI_IMPULSE_OK) {
    Serial.printf("run_classifier error: %d\n", r);
    delay(100);
    return;
  }

  /* 4. Debug: print inference time */
  Serial.printf("[Perf] Inference time: %lu us\n", inference_time_us);

  /* 5. Print all class scores */
  Serial.println("Inference results:");
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.printf("  %s: %.3f\n",
                  result.classification[i].label,
                  result.classification[i].value);
  }

  /* 6. Determine top class */
  int top_i = 0;
  float top_v = 0.f;
  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > top_v) {
      top_v = result.classification[i].value;
      top_i = i;
    }
  }
  const char* top_label = result.classification[top_i].label;
  Serial.printf("Top: %s (%.3f)\n", top_label, top_v);

  /* 7. Trigger action if above threshold */
  const float CONFIDENCE_THRESHOLD = 0.75f;
  if (top_v >= CONFIDENCE_THRESHOLD) {
    handleIntent(top_label, top_v);
  } else {
    Serial.println("Confidence below threshold, no action.");
  }
}
/* === KWS Inference === */

/* === Handle Intent Actions ===  */

void handleIntent(const char* label, float confidence) {
  Serial.printf("Triggering action for: %s (%.3f)\n", label, confidence);
  DateTime now = rtc.now();

  /* Display on OLED */
  char line1[32], line2[32];
  snprintf(line1, sizeof(line1), "%02d:%02d %s", now.hour(), now.minute(), label);
  snprintf(line2, sizeof(line2), "Conf: %.2f", confidence);
  showOnOLED(line1, line2);

  /* Buzzer feedback */
  buzz(100);

  /* Read sensors via I2C expander and VL53L0X */
  float distance = readDistanceCM();
  bool hall = readHallViaExpander();
  Serial.printf(" Distance: %.1f cm, Hall: %d\n", distance, hall);
 
  /* Common logging */
  postToDashboard(label, confidence);
  logToSD(label, confidence, now);

  /* Label-specific actions */
  if (strcmp(label, "ask_time") == 0) {
    // Play time via speaker or skip if no speaker
    Serial.println("ask_time action: play time (not implemented)");
  }
  else if (strcmp(label, "notify_sos") == 0) {
    sendEmail("SOS Alert", "User requested SOS at pill reminder.");
    Serial.println("notify_sos action: email sent");
  }
  else if (strcmp(label, "confirm_taken") == 0) {
    Serial.println("confirm_taken action logged");
  }
  else if (strcmp(label, "deny_taken") == 0) {
    Serial.println("deny_taken action logged");
  }
  else if (strcmp(label, "remind_later") == 0) {
    Serial.println("remind_later action: schedule next reminder (not implemented)");
  }
  else if (strcmp(label, "ask_med_details") == 0) {
    Serial.println("ask_med_details action: play details (not implemented)");
  }
  else if (strcmp(label, "ask_schedule") == 0) {
    Serial.println("ask_schedule action: play schedule (not implemented)");
  }
  else if (strcmp(label, "irrelevant") == 0) {
    Serial.println("irrelevant detected: ignoring.");
  }
}
/* === Handle Intent Actions ===  */


/* === Utility Functions ===  */

/* Send Email via SMTP (simplified; fill base64 creds as needed)*/
void sendEmail(const char* subject, const char* body) {
  Serial.println("Sending email...");
  WiFiClientSecure client;
  client.setInsecure(); // for testing
  if (!client.connect(SMTP_SERVER, SMTP_PORT)) {
    Serial.println("SMTP connect failed");
    return;
  }
  // Wait for 220 greeting
  if (client.readStringUntil('\n').indexOf("220") < 0) {
    Serial.println("SMTP no 220 greeting");
    client.stop();
    return;
  }
  // EHLO
  client.printf("EHLO %s\r\n", WIFI_SSID);
  client.readStringUntil('\n');
  // AUTH LOGIN
  client.printf("AUTH LOGIN\r\n");
  client.readStringUntil('\n');
  // USER (base64)
  client.printf("%s\r\n", EMAIL_USER);
  client.readStringUntil('\n');
  // PASS (base64)
  client.printf("%s\r\n", EMAIL_PASS);
  client.readStringUntil('\n');
  // MAIL FROM
  client.printf("MAIL FROM:<%s>\r\n", EMAIL_USER);
  client.readStringUntil('\n');
  // RCPT TO
  client.printf("RCPT TO:<%s>\r\n", EMAIL_TO);
  client.readStringUntil('\n');
  // DATA
  client.printf("DATA\r\n");
  client.readStringUntil('\n');
  // Headers & body
  client.printf("Subject: %s\r\n", subject);
  client.printf("To: %s\r\n", EMAIL_TO);
  client.printf("From: %s\r\n", EMAIL_USER);
  client.printf("\r\n");
  client.printf("%s\r\n", body);
  client.printf(".\r\n");
  client.readStringUntil('\n');
  // QUIT
  client.printf("QUIT\r\n");
  client.readStringUntil('\n');
  client.stop();
  Serial.println("Email sent");
}


/* === Utility Functions ===  */

/* ============== POST to Dashboard ========== */
void postToDashboard(const char* label, float confidence) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected: cannot post dashboard");
    return;
  }
  HTTPClient http;
  http.begin(DASHBOARD_URL);
  http.addHeader("Content-Type", "application/json");
  DateTime now = rtc.now();
  char buf[64];
  snprintf(buf, sizeof(buf),
           "{\"timestamp\":\"%04d-%02d-%02dT%02d:%02d:%02dZ\",\"label\":\"%s\",\"confidence\":%.3f}",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second(),
           label, confidence);
  int code = http.POST(buf);
  Serial.printf("Dashboard POST code: %d\n", code);
  http.end();
}
/* ============== POST to Dashboard ========== */

/* ============== Log to SD Card ========== */
void logToSD(const char* label, float confidence, DateTime timestamp) {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD not available");
    return;
  }
  File file = SD.open("/log.csv", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open log.csv");
    return;
  }
  float distance = readDistanceCM();
  bool hall = readHallViaExpander();
  char line[128];
  snprintf(line, sizeof(line),
           "%04d-%02d-%02d %02d:%02d:%02d,%s,%.3f,%.1f,%d",
           timestamp.year(), timestamp.month(), timestamp.day(),
           timestamp.hour(), timestamp.minute(), timestamp.second(),
           label, confidence, distance, hall ? 1 : 0);
  file.println(line);
  file.close();
  Serial.println("Logged to SD");
}
/* ============== Log to SD Card ========== */


/*  Read distance via VL53L0X */
float readDistanceCM() {
  uint16_t dist = vl53.readRangeSingleMillimeters();
  if (vl53.timeoutOccurred()) {
    return -1.0;
  }
  return dist / 10.0; // mm to cm
}

// Read hall sensor via PCF8574
bool readHallViaExpander() {
  int v = expander.digitalRead(EXP_PIN_HALL);
  return (v == HIGH);
}

// Buzzer feedback
void buzz(int ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
}

// Display helper
void showOnOLED(const char* line1, const char* line2) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(line1);
  if (line2) display.println(line2);
  display.display();
}
