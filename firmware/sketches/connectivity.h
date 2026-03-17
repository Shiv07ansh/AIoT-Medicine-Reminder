/*
 * connectivity.h
 * =====================================================================
 * WiFi, SMTP email, HTTP dashboard POST
 * Credentials are in credentials.h
 * =====================================================================
 */

#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "credentials.h"

// =====================================================================
// WiFi
// =====================================================================
void initWiFi() {
  Serial.printf("[WIFI] Connecting to %s...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - t0 > 12000) {
      Serial.println("\n[WIFI] Timeout — continuing offline.");
      return;
    }
  }
  Serial.print("\n[WIFI] Connected. IP: ");
  Serial.println(WiFi.localIP());
}

// =====================================================================
// SMTP Email (raw TCP — basic AUTH LOGIN)
// For production: use ESP_Mail_Client library instead.
// =====================================================================
void sendEmail(const char* subject, const char* body) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[EMAIL] WiFi not connected — skipping.");
    return;
  }
  Serial.printf("[EMAIL] Sending: %s\n", subject);

  WiFiClientSecure client;
  client.setInsecure();   // TODO: use setCACert() in production

  if (!client.connect(SMTP_SERVER, SMTP_PORT)) {
    Serial.println("[EMAIL] SMTP connect failed.");
    return;
  }

  // Helper lambda: send + drain one line
  auto smtp = [&](const char* cmd) {
    client.printf("%s\r\n", cmd);
    client.readStringUntil('\n');
  };

  client.readStringUntil('\n');           // 220 greeting
  smtp("EHLO esp32");
  smtp("AUTH LOGIN");
  smtp(EMAIL_USER_B64);                   // base64-encoded user
  smtp(EMAIL_PASS_B64);                   // base64-encoded password
  client.printf("MAIL FROM:<%s>\r\n", EMAIL_FROM); client.readStringUntil('\n');
  client.printf("RCPT TO:<%s>\r\n",   EMAIL_TO);   client.readStringUntil('\n');
  smtp("DATA");
  client.printf("Subject: %s\r\nTo: %s\r\nFrom: %s\r\n\r\n%s\r\n.\r\n",
                subject, EMAIL_TO, EMAIL_FROM, body);
  client.readStringUntil('\n');
  smtp("QUIT");
  client.stop();
  Serial.println("[EMAIL] Sent.");
}

// =====================================================================
// HTTP Dashboard POST
// Expects a JSON endpoint at DASHBOARD_URL.
// NOTE: WORK-IN-PROGRESS, needs a lot more thorough design implementation
// =====================================================================
void postToDashboard(const char* label, float confidence) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[DASH] WiFi not connected — skipping.");
    return;
  }
  HTTPClient http;
  http.begin(DASHBOARD_URL);
  http.addHeader("Content-Type", "application/json");

  char body[128];
  snprintf(body, sizeof(body),
           "{\"label\":\"%s\",\"confidence\":%.3f}",
           label, confidence);

  int code = http.POST(body);
  Serial.printf("[DASH] POST → %d\n", code);
  http.end();
}
