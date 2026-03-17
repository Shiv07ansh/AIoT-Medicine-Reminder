/*
 * sensors.h
 * =====================================================================
 * Peripheral helpers:
 *   - OLED display (SSD1306 via I2C)
 *   - Buzzer (GPIO, active-low or active-high)
 *   - Status LED (GPIO)
 *   - Hall effect sensor (GPIO or PCF8574 expander)
 *   - Ultrasonic sensor HC-SR04
 * =====================================================================
 */

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// =====================================================================
// PIN CONFIG — adjust to your wiring
// =====================================================================
#define PIN_BUZZER          15
#define PIN_LED             2

// Ultrasonic HC-SR04
#define PIN_TRIG            12
#define PIN_ECHO            14

// Hall sensor (direct GPIO, LOW = magnet present = lid closed)
#define PIN_HALL            13

// OLED
#define OLED_WIDTH          128
#define OLED_HEIGHT         64
#define OLED_I2C_ADDR       0x3C

// Hand detection threshold (cm).
// Set to d_empty_box - 1.0 cm at calibration time.
// TODO: replace with dynamic calibration on boot.
#define HAND_DETECT_THRESHOLD_CM  10.0f

// Buzzer patterns
enum BuzzPattern { BUZZ_SUCCESS, BUZZ_SOS, BUZZ_ALERT };

// =====================================================================
// OLED object (file-scoped)
// =====================================================================
static Adafruit_SSD1306 _oled(OLED_WIDTH, OLED_HEIGHT, &Wire);

// =====================================================================
// INIT
// =====================================================================
void initSensors() {
  // Buzzer + LED
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Ultrasonic
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  // Hall
  pinMode(PIN_HALL, INPUT_PULLUP);

  // OLED
  Wire.begin();
  if (!_oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println("[SENSORS] OLED init failed.");
  } else {
    _oled.clearDisplay();
    _oled.setTextSize(1);
    _oled.setTextColor(SSD1306_WHITE);
    _oled.display();
    Serial.println("[SENSORS] OLED OK.");
  }

  Serial.println("[SENSORS] Peripherals initialized.");
}

// =====================================================================
// OLED HELPERS
// =====================================================================
void showOLED(const char* line1, const char* line2 = nullptr) {
  _oled.clearDisplay();
  _oled.setTextSize(1);
  _oled.setCursor(0, 0);
  _oled.println(line1);
  if (line2 && strlen(line2) > 0) {
    _oled.setCursor(0, 16);
    _oled.println(line2);
  }
  _oled.display();
}

// =====================================================================
// LED
// =====================================================================
void setLED(bool on) {
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

// =====================================================================
// BUZZER
// =====================================================================
void buzzOnce(int durationMs) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(durationMs);
  digitalWrite(PIN_BUZZER, LOW);
}

void buzzPattern(BuzzPattern p) {
  switch (p) {
    case BUZZ_SUCCESS:
      // Two short beeps
      buzzOnce(100); delay(100);
      buzzOnce(100); delay(100);
      buzzOnce(300);
      break;
    case BUZZ_SOS:
      // Three short, three long, three short
      for (int i = 0; i < 3; i++) { buzzOnce(100); delay(100); }
      for (int i = 0; i < 3; i++) { buzzOnce(400); delay(100); }
      for (int i = 0; i < 3; i++) { buzzOnce(100); delay(100); }
      break;
    case BUZZ_ALERT:
      // Three sharp beeps
      for (int i = 0; i < 3; i++) { buzzOnce(200); delay(200); }
      break;
  }
}

// =====================================================================
// HALL SENSOR
// Returns true if lid is OPEN (magnet absent → pin HIGH with pull-up)
// =====================================================================
bool readHallSensor() {
  // Assumes NO (normally open) switch: LOW = magnet present = lid closed
  // HIGH = magnet absent = lid open
  return (digitalRead(PIN_HALL) == HIGH);
}

// =====================================================================
// ULTRASONIC (HC-SR04)
// Returns distance in cm, or -1.0 on timeout
// =====================================================================
float readUltrasonic() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, 30000); // 30ms timeout ~5m
  if (duration == 0) return -1.0f;
  return (duration * 0.0343f) / 2.0f;
}
