// AIPool app — Phase 3: valve + pump + VGA gauge + QR provisioning.
// Spec: docs/aipool-spec.md. QR via shared qrscan lib (src/qrscan.c over src/quirc/, MIT).
// Controls-safety: see valve.cpp contract + controlUpdate pre-hold/cap.

#include <Arduino.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

#include "app_config.h"
#include "app_state.h"
#include "bus.h"
#include "provisioning.h"
#include "pump.h"
#include "valve.h"
#include "vision.h"

#define LED_PIN 33 // ESP32CAM onboard LED (active LOW)
#define PRE_HOLD_MS 15000
#define BACKWASH_MIN_MAX 10

HardwareSerial RS485Bus(2);
SemaphoreHandle_t g_busMutex = nullptr;

static Preferences s_prefs;
static WebServer s_server(80);
static AppConfig g_cfg;
static AppState g_state;
static ControlState g_ctrl;
static unsigned long g_bootMs = 0;
static unsigned long g_lastTickMs = 0;

// ---------- NVS ----------
static void configLoad() {
  g_cfg = appConfigDefaults();
  if (!s_prefs.begin(AIPOOL_NVS_NS, true))
    return;
  String ssid = s_prefs.getString(AIPOOL_NVS_SSID, "");
  String pass = s_prefs.getString(AIPOOL_NVS_PASS, "");
  String host = s_prefs.getString(AIPOOL_NVS_HOST, "aipool");
  // Precedence: NVS (if set) > build-time AIPOOL_WIFI_* defaults > empty.
  // An unset NVS ssid must NOT wipe a baked-in default; an explicitly
  // stored ssid (even with empty pass = open network) wins entirely.
  if (ssid.length() > 0) {
    ssid.toCharArray(g_cfg.wifi_ssid, sizeof(g_cfg.wifi_ssid));
    pass.toCharArray(g_cfg.wifi_pass, sizeof(g_cfg.wifi_pass));
  }
  host.toCharArray(g_cfg.hostname, sizeof(g_cfg.hostname));
  g_cfg.threshold_psi = s_prefs.getFloat(AIPOOL_NVS_THRESH, g_cfg.threshold_psi);
  g_cfg.backwash_min =
      (uint8_t)s_prefs.getUChar(AIPOOL_NVS_BWMIN, g_cfg.backwash_min);
  g_cfg.target_rpm = s_prefs.getUShort(AIPOOL_NVS_RPM, g_cfg.target_rpm);
  s_prefs.end();
  if (g_cfg.backwash_min < 1)
    g_cfg.backwash_min = 1;
  if (g_cfg.backwash_min > BACKWASH_MIN_MAX)
    g_cfg.backwash_min = BACKWASH_MIN_MAX;
  if (g_cfg.target_rpm < 600)
    g_cfg.target_rpm = 600;
  if (g_cfg.target_rpm > 3450)
    g_cfg.target_rpm = 3450;
}

static void configSave() {
  if (!s_prefs.begin(AIPOOL_NVS_NS, false))
    return;
  s_prefs.putString(AIPOOL_NVS_SSID, g_cfg.wifi_ssid);
  s_prefs.putString(AIPOOL_NVS_PASS, g_cfg.wifi_pass);
  s_prefs.putString(AIPOOL_NVS_HOST, g_cfg.hostname);
  s_prefs.putFloat(AIPOOL_NVS_THRESH, g_cfg.threshold_psi);
  s_prefs.putUChar(AIPOOL_NVS_BWMIN, g_cfg.backwash_min);
  s_prefs.putUShort(AIPOOL_NVS_RPM, g_cfg.target_rpm);
  s_prefs.end();
}

// ---------- control ----------
// Safety contract (controls-safety reviewed): valve moves only after
// pump_stopped_ok() + 15s pre-hold; break-before-make + echo verify live
// in valve.cpp; 10-min cap enforced below from BACKWASH_RUN entry.
static uint32_t backwashCapMs() {
  return (uint32_t)g_cfg.backwash_min * 60UL * 1000UL;
}

static void enterMoving(PoolMode target) {
  g_state.mode = PoolMode::MOVING;
  g_ctrl.preHoldActive = true;
  g_ctrl.preHoldStartMs = millis();
  g_ctrl.pendingTarget = target;
  pumpStop(); // fail-safe: de-energize pump first; pre-hold runs regardless
}

static void requestMode(PoolMode target) {
  if (target != PoolMode::FILTER && target != PoolMode::BACKWASH)
    return;
  PoolMode stable =
      (g_state.mode == PoolMode::MOVING) ? g_ctrl.pendingTarget : g_state.mode;
  if (stable == target && !g_ctrl.backwashActive)
    return;
  // Safety: any motion goes through the 15s pre-hold.
  g_ctrl.backwashActive = false;
  enterMoving(target);
}

static void controlUpdate() {
  uint32_t now = millis();
  if (g_state.mode == PoolMode::MOVING && g_ctrl.preHoldActive) {
    if ((now - g_ctrl.preHoldStartMs >= PRE_HOLD_MS) && pumpStoppedOk()) {
      g_ctrl.preHoldActive = false;
      PoolMode target = g_ctrl.pendingTarget;
      bool moved = (target == PoolMode::BACKWASH) ? valveGotoBackwash()
                                                  : valveGotoFilter();
      if (!moved) {
        g_state.mode = PoolMode::FAULT;
        g_state.fault = valveLastError();
        g_ctrl.backwashActive = false;
        return;
      }
      g_state.valve = (target == PoolMode::BACKWASH) ? "backwash" : "filter";
      g_state.mode = target;
      g_state.fault = "none";
      if (target == PoolMode::BACKWASH) {
        g_ctrl.backwashActive = true;
        g_ctrl.backwashStartMs = now;
        if (!pumpRun(g_cfg.target_rpm)) {
          g_state.mode = PoolMode::FAULT;
          g_state.fault = pumpLastError();
          g_ctrl.backwashActive = false;
          return;
        }
        g_state.pump = "running";
      } else {
        g_ctrl.backwashActive = false;
        g_state.pump = "stopped";
      }
    }
  }
  if (g_state.mode == PoolMode::BACKWASH && g_ctrl.backwashActive) {
    if (now - g_ctrl.backwashStartMs >= backwashCapMs()) {
      // Hard kill: revert to FILTER through the same safe path.
      requestMode(PoolMode::FILTER);
    }
  }
}

// ---------- HTTP ----------
static String stateJson() {
  char buf[320];
  snprintf(buf, sizeof(buf),
           "{\"psi\":%.2f,\"rpm\":%d,\"watts\":%d,\"mode\":\"%s\","
           "\"valve\":\"%s\",\"pump\":\"%s\",\"fault\":\"%s\","
           "\"threshold_psi\":%.2f,\"backwash_min\":%u,\"target_rpm\":%u}",
           g_state.psi, g_state.rpm, g_state.watts,
           poolModeStr(g_state.mode), g_state.valve, g_state.pump,
           g_state.fault, g_cfg.threshold_psi, g_cfg.backwash_min,
           g_cfg.target_rpm);
  return String(buf);
}

static void handleState() {
  s_server.send(200, "application/json", stateJson());
}

static void handleHealth() {
  char buf[128];
  snprintf(buf, sizeof(buf), "{\"ok\":true,\"up_s\":%lu,\"mode\":\"%s\"}",
           (millis() - g_bootMs) / 1000, poolModeStr(g_state.mode));
  s_server.send(200, "application/json", buf);
}

static void handleGauge() {
  const uint8_t *frame = visionFrame();
  if (!frame) {
    s_server.send(503, "text/plain", "no frame yet");
    return;
  }
  // Serve latest VGA gray frame as 8-bit BMP (browsers render it directly).
  // 640x480 rows are 640 bytes (already 4-aligned), bottom-up order.
  const uint32_t W = 640, H = 480;
  const uint32_t paletteSize = 256 * 4;
  const uint32_t offBits = 14 + 40 + paletteSize;
  const uint32_t fileSize = offBits + W * H;
  uint8_t hdr[14 + 40];
  memset(hdr, 0, sizeof(hdr));
  hdr[0] = 'B';
  hdr[1] = 'M';
  hdr[2] = (uint8_t)(fileSize & 0xFF);
  hdr[3] = (uint8_t)((fileSize >> 8) & 0xFF);
  hdr[4] = (uint8_t)((fileSize >> 16) & 0xFF);
  hdr[5] = (uint8_t)((fileSize >> 24) & 0xFF);
  hdr[10] = (uint8_t)(offBits & 0xFF);
  hdr[11] = (uint8_t)((offBits >> 8) & 0xFF);
  hdr[12] = (uint8_t)((offBits >> 16) & 0xFF);
  hdr[13] = (uint8_t)((offBits >> 24) & 0xFF);
  hdr[14] = 40; // info header size
  hdr[18] = (uint8_t)(W & 0xFF);
  hdr[19] = (uint8_t)((W >> 8) & 0xFF);
  hdr[22] = (uint8_t)(H & 0xFF);
  hdr[23] = (uint8_t)((H >> 8) & 0xFF);
  hdr[26] = 1; // planes
  hdr[28] = 8; // bits per pixel
  s_server.setContentLength(fileSize);
  s_server.send(200, "image/bmp", "");
  WiFiClient client = s_server.client();
  client.write(hdr, sizeof(hdr));
  uint8_t pal[256 * 4];
  for (int i = 0; i < 256; i++) {
    pal[i * 4 + 0] = (uint8_t)i;
    pal[i * 4 + 1] = (uint8_t)i;
    pal[i * 4 + 2] = (uint8_t)i;
    pal[i * 4 + 3] = 0;
  }
  client.write(pal, sizeof(pal));
  for (int y = (int)H - 1; y >= 0; y--) {
    client.write(frame + (size_t)y * W, W);
  }
}

// Minimal JSON field extraction without extra libs (Phase 0).
static bool extractJsonString(const String &body, const char *key, char *out,
                              size_t outCap) {
  String q = String("\"") + key + String("\"");
  int i = body.indexOf(q);
  if (i < 0)
    return false;
  int c = body.indexOf(':', (unsigned)i + q.length());
  if (c < 0)
    return false;
  int q1 = body.indexOf('"', (unsigned)c + 1);
  if (q1 < 0)
    return false;
  int q2 = body.indexOf('"', (unsigned)q1 + 1);
  if (q2 < 0)
    return false;
  String v = body.substring((unsigned)q1 + 1, (unsigned)q2);
  v.toCharArray(out, outCap);
  return true;
}

static bool extractJsonFloat(const String &body, const char *key, float &out) {
  String q = String("\"") + key + String("\"");
  int i = body.indexOf(q);
  if (i < 0)
    return false;
  int c = body.indexOf(':', (unsigned)i + q.length());
  if (c < 0)
    return false;
  out = body.substring((unsigned)c + 1).toFloat();
  return true;
}

static bool extractJsonInt(const String &body, const char *key, long &out) {
  String q = String("\"") + key + String("\"");
  int i = body.indexOf(q);
  if (i < 0)
    return false;
  int c = body.indexOf(':', (unsigned)i + q.length());
  if (c < 0)
    return false;
  out = body.substring((unsigned)c + 1).toInt();
  return true;
}

static void handleCmd() {
  if (s_server.method() != HTTP_POST) {
    s_server.send(405, "text/plain", "POST only");
    return;
  }
  String body = s_server.arg("plain");
  if (body.length() == 0) {
    s_server.send(400, "text/plain", "empty body");
    return;
  }
  char modeStr[16] = {0};
  float thresh = 0;
  long bwmin = -1, rpm = -1;
  bool hasMode = extractJsonString(body, "mode", modeStr, sizeof(modeStr));
  bool hasThresh = extractJsonFloat(body, "threshold_psi", thresh);
  bool hasBw = extractJsonInt(body, "backwash_min", bwmin);
  bool hasRpm = extractJsonInt(body, "rpm", rpm);

  if (hasThresh)
    g_cfg.threshold_psi = thresh;
  if (hasBw) {
    if (bwmin < 1 || bwmin > BACKWASH_MIN_MAX) {
      s_server.send(400, "text/plain", "backwash_min must be 1..10");
      return;
    }
    g_cfg.backwash_min = (uint8_t)bwmin;
  }
  if (hasRpm) {
    if (rpm < 600 || rpm > 3450) {
      s_server.send(400, "text/plain", "rpm must be 600..3450");
      return;
    }
    g_cfg.target_rpm = (uint16_t)rpm;
  }
  if (hasMode) {
    if (strcmp(modeStr, "backwash") == 0)
      requestMode(PoolMode::BACKWASH);
    else if (strcmp(modeStr, "filter") == 0)
      requestMode(PoolMode::FILTER);
    else {
      s_server.send(400, "text/plain", "mode must be filter|backwash");
      return;
    }
  }
  configSave();
  s_server.send(200, "application/json", stateJson());
}

static void routesInit() {
  s_server.on("/health", HTTP_GET, handleHealth);
  s_server.on("/api/state", HTTP_GET, handleState);
  s_server.on("/api/cmd", HTTP_POST, handleCmd);
  s_server.on("/gauge", HTTP_GET, handleGauge);
  s_server.onNotFound(
      []() { s_server.send(404, "text/plain", "not found"); });
}

// ---------- net ----------
// Boot net flow: NVS creds -> QR scan (15s, LED blink) retry forever.
// No fallback AP: stays in STA + QR-scan until provisioned.
static bool wifiConnectAttempt(unsigned long timeoutMs = 10000) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_cfg.wifi_ssid, g_cfg.wifi_pass);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
    delay(200);
  }
  return WiFi.status() == WL_CONNECTED;
}

static void wifiInit() {
  for (;;) {
    if (strlen(g_cfg.wifi_ssid) > 0) {
      Serial.println("[net] trying NVS credentials...");
      if (wifiConnectAttempt()) {
        Serial.printf("[net] connected: %s\n",
                      WiFi.localIP().toString().c_str());
        return;
      }
      Serial.println("[net] NVS credentials failed, entering QR scan...");
    } else {
      Serial.println("[net] no NVS credentials, entering QR scan...");
    }
    char payload[256];
    if (provScanQr(PROV_SCAN_TIMEOUT_MS, payload, sizeof(payload))) {
      char ssid[64], pass[64];
      if (provParseWifi(payload, ssid, sizeof(ssid), pass, sizeof(pass))) {
        Serial.printf("[net] QR parsed ssid=%s, connecting...\n", ssid);
        strncpy(g_cfg.wifi_ssid, ssid, sizeof(g_cfg.wifi_ssid) - 1);
        strncpy(g_cfg.wifi_pass, pass, sizeof(g_cfg.wifi_pass) - 1);
        configSave();
        if (wifiConnectAttempt()) {
          Serial.printf("[net] connected: %s\n",
                        WiFi.localIP().toString().c_str());
          return;
        }
        Serial.println("[net] QR credentials failed, retrying scan...");
      } else {
        Serial.println("[net] QR payload not WIFI:.. format, retrying scan...");
      }
    } else {
      Serial.println("[net] QR scan timeout, retrying scan...");
    }
  }
}

// ---------- USB debug (bench HIL; mirrors tests/esp32-combined-test keys)
static String s_line;

static void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      s_line.trim();
      if (s_line == "state") {
        Serial.println(stateJson());
      } else if (s_line == "backwash") {
        requestMode(PoolMode::BACKWASH);
        Serial.println("[usb] backwash requested");
      } else if (s_line == "filter") {
        requestMode(PoolMode::FILTER);
        Serial.println("[usb] filter requested");
      } else if (s_line == "poll") {
        PumpReading pr;
        if (pumpPoll(pr)) {
          Serial.printf("[usb] pump %s rpm=%u demand=%u watts=%u\n",
                        century_status_str(pr.status), pr.rpm, pr.demand,
                        pr.watts);
        } else {
          Serial.printf("[usb] pump no reply (%s)\n", pumpLastError());
        }
        VisionReading v = visionLatest();
        Serial.printf("[usb] gauge psi=%.2f conf=%.2f support=%.3f ok=%d\n",
                      v.psi, v.confSigma, v.support, (int)v.ok);
      } else if (s_line.length() > 0) {
        Serial.println("[usb] keys: state backwash filter poll");
      }
      s_line = "";
    } else {
      s_line += c;
      if (s_line.length() > 32)
        s_line = "";
    }
  }
}

// ---------- arduino ----------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);
  delay(300);
  g_bootMs = millis();

  Serial.println("\n=== AIPool app (Phase 3: valve+pump+gauge+qr) ===");
  configLoad();
  busInit(); // shared RS485 mutex + UART2 @9600
  pumpInit(g_cfg.target_rpm);
  if (!valveInit()) {
    Serial.printf("[valve] fail-safe init failed: %s\n", valveLastError());
    g_state.mode = PoolMode::FAULT;
    g_state.fault = valveLastError();
  }
  // Camera first: provScanQr() inside wifiInit() borrows the same VGA
  // grayscale stream via qrscan_decode — no second esp_camera_init.
  if (!visionInit()) {
    Serial.printf("[vision] init failed: %s (psi stays 0)\n",
                  visionLastError());
  } else {
    Serial.println("[vision] VGA grayscale ready");
  }

  wifiInit();
  if (!MDNS.begin(g_cfg.hostname)) {
    Serial.println("[net] mDNS failed");
  } else {
    Serial.printf("[net] mDNS: http://%s.local\n", g_cfg.hostname);
  }
  routesInit();
  s_server.begin();

  g_state.mode = PoolMode::FILTER;
  g_state.valve = "filter";
  g_state.pump = "stopped";
  Serial.println("[app] ready: GET /health /api/state, POST /api/cmd");
}

void loop() {
  s_server.handleClient();
  handleSerial();
  controlUpdate();
  if (g_state.mode == PoolMode::BACKWASH) {
    pumpRefreshIfDue(g_cfg.target_rpm); // 60s demand re-arm
  }
  if (millis() - g_lastTickMs >= 1000) {
    g_lastTickMs = millis();
    if (visionPoll()) {
      VisionReading v = visionLatest();
      g_state.psi = v.psi;
      g_state.conf_sigma = v.confSigma;
      g_state.support = v.support;
    }
    PumpReading pr;
    if (pumpPoll(pr)) {
      g_state.rpm = pr.rpmOk ? (int)pr.rpm : 0;
      g_state.watts = pr.wattsOk ? (int)pr.watts : 0;
      if (pr.status == CENTURY_STATUS_FAULT) {
        g_state.mode = PoolMode::FAULT;
        g_state.fault = century_fault_str(pr.faultCode);
      }
    }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // 1Hz alive blink
  }
  delay(5);
}
