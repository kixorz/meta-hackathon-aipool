#pragma once

// NVS-backed runtime config. Defaults mirror docs/aipool-spec.md.
// Phase 1 drivers read these; POST /api/cmd writes + persists them.

#include <stdint.h>
#include <string.h>

// Build-time WiFi defaults from platformio.ini (env AIPOOL_WIFI_SSID/_PASS).
// Empty when the env vars are unset — then QR-scan/AP fallback applies.
#ifndef AIPOOL_WIFI_SSID
#define AIPOOL_WIFI_SSID ""
#endif
#ifndef AIPOOL_WIFI_PASS
#define AIPOOL_WIFI_PASS ""
#endif

struct AppConfig {
  char wifi_ssid[64];
  char wifi_pass[64];
  char hostname[32];
  float threshold_psi;
  uint8_t backwash_min; // 1..10 hard cap
  uint16_t target_rpm;  // 600..3450
};

inline AppConfig appConfigDefaults() {
  AppConfig c = {};
  strncpy(c.wifi_ssid, AIPOOL_WIFI_SSID, sizeof(c.wifi_ssid) - 1);
  strncpy(c.wifi_pass, AIPOOL_WIFI_PASS, sizeof(c.wifi_pass) - 1);
  c.hostname[0] = 'a';
  c.hostname[1] = 'i';
  c.hostname[2] = 'p';
  c.hostname[3] = 'o';
  c.hostname[4] = 'o';
  c.hostname[5] = 'l';
  c.hostname[6] = '\0';
  c.threshold_psi = 25.0f;
  c.backwash_min = 8;
  c.target_rpm = 1600;
  return c;
}

// NVS namespace + keys (single source; keep in sync with main.cpp).
#define AIPOOL_NVS_NS "aipool"
#define AIPOOL_NVS_SSID "wifi_ssid"
#define AIPOOL_NVS_PASS "wifi_pass"
#define AIPOOL_NVS_HOST "hostname"
#define AIPOOL_NVS_THRESH "thresh_psi"
#define AIPOOL_NVS_BWMIN "bw_min"
#define AIPOOL_NVS_RPM "target_rpm"
