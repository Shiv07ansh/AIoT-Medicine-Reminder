/*
 * credentials.h
 * =====================================================================
 * ALL secrets live here.
 * =====================================================================
 */

#pragma once

// --- WiFi ---
#define WIFI_SSID         "YOUR_SSID"
#define WIFI_PASS         "YOUR_WIFI_PASSWORD"

// --- SMTP ---
// Use port 465 (SSL) or 587 (STARTTLS).
// EMAIL_USER_B64 / EMAIL_PASS_B64 = base64 of your credentials.
// Generate with: python3 -c "import base64; print(base64.b64encode(b'user@example.com').decode())"
#define SMTP_SERVER       "smtp.gmail.com"
#define SMTP_PORT         465
#define EMAIL_FROM        "your_device@gmail.com"
#define EMAIL_TO          "caregiver@example.com"
#define EMAIL_USER_B64    "eW91cl9kZXZpY2VAZ21haWwuY29t"   // base64 of EMAIL_FROM
#define EMAIL_PASS_B64    "eW91cl9hcHBfcGFzc3dvcmQ="       // base64 of app password

// --- Dashboard ---
#define DASHBOARD_URL     "https://your-server.example.com/api/log"
