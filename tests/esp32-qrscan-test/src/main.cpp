// ESP32CAM QR scan test
// Points the OV2640 (VGA grayscale) at a printed QR code and decodes it
// through the shared qrscan driver, printing per-frame diagnostics.
// Intended as the bring-up rig for the WiFi-provisioning scan: same camera
// path, same decode path, no WiFi/NVS/HTTP involved.
//
// Reuses the shared driver in src/qrscan.c + include/qrscan.h (thin quirc
// wrapper: grayscale copy, decode + mirror retry, half-res + 1px-jiggle
// second chances) over the vendored backend in src/quirc/ + src/openmv/
// (MIT, via ESP32QRCodeReader). All are symlinked into this test's src/
// so PlatformIO compiles them as-is; fix protocol/decode bugs in src/
// and include/, not here.
//
// Camera: AI Thinker ESP32-CAM, OV2640, PIXFORMAT_GRAYSCALE, FRAMESIZE_VGA
// (640x480). No RS485 wiring needed for this test.
// Printed target: any QR, e.g. a WIFI:.. code from the provisioning flow;
// hold it 15-25 cm in front of the lens, flat, lit, filling 1/4+ of frame.
//
// USB debug console = Serial @115200 on /dev/cu.usbserial-1440. Keys:
//   C = toggle continuous scan   S = single capture + decode snapshot
//   D = ascii thumbnail of last frame (#=dark)
//   F = full-res grayscale frame as hex (host -> PGM, shows quirc's input)
//   L = flash manual toggle (auto off)   A = flash auto on/off
//   V = toggle sensor vflip
//   M = toggle sensor hmirror (V+M = 180-degree rotation)   H = help
// The loop never blocks on USB; continuous scan runs by default.
//
// Workspace warning (see qrscan.h): qr_workspace_t is ~39KB — s_ws below
// is static storage, never the stack (loopTask is 8KB).

#include <Arduino.h>
#include <esp_camera.h>

#include "qrscan.h"

// ---------- camera pins (AI Thinker ESP32-CAM, same as firmware vision) ----------
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_Y9 35
#define CAM_PIN_Y8 34
#define CAM_PIN_Y7 39
#define CAM_PIN_Y6 36
#define CAM_PIN_Y5 21
#define CAM_PIN_Y4 19
#define CAM_PIN_Y3 18
#define CAM_PIN_Y2 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#define QR_W 640
#define QR_H 480
#define SCAN_PAUSE_MS 500 // pause between continuous-scan frames
#define LED_PIN 33        // ESP32CAM onboard LED (active LOW)
#define FLASH_PIN 4       // ESP32CAM white flash LED next to the lens (HIGH = on)
// Sensor orientation for this rig: the module reads out vertically flipped
// natively, so vflip alone restores the true image (vflip+hmirror produced
// a horizontally mirrored stream — confirmed by finder template-match).
// V/M keys toggle live.
#define CAM_VFLIP_INIT 1
#define CAM_HMIRROR_INIT 0

// ~39KB: static, never stack.
static qr_workspace_t s_ws;
static bool s_continuous = true;
static bool s_flash = true; // flash on at boot: the QR target needs light
static bool s_flashAuto = true; // auto mode: LED follows ambient (A toggles)
static int s_vflip = CAM_VFLIP_INIT;
static int s_hmirror = CAM_HMIRROR_INIT;
static unsigned long s_frameN = 0;
static String s_line;

// ---------- flash auto-exposure ----------
// The LED follows ambient light measured from the captured frame itself
// (stride-sampled mean gray). Decisions use UNLIT frames only: a dark unlit
// frame turns the LED on; every FLASH_AUTO_PROBE_EVERY lit frames the LED
// goes off for FLASH_AUTO_PROBE_N frames (AEC settle time) and the last
// probe frame decides — bright stays off, dark turns back on. Because lit
// frames never influence a decision, the LED cannot oscillate.
#define FLASH_AUTO_DARK_MEAN 70   // unlit mean below this: need light
#define FLASH_AUTO_PROBE_EVERY 10 // lit frames between ambient probes
#define FLASH_AUTO_PROBE_N 4      // unlit probe frames per probe
static int s_probeIn = FLASH_AUTO_PROBE_EVERY; // lit frames until next probe
static int s_probing = 0;                      // unlit probe frames remaining

static int frameMean(const uint8_t *buf, size_t n) {
  uint32_t acc = 0;
  size_t cnt = 0;
  for (size_t i = 0; i < n; i += 16) {
    acc += buf[i];
    cnt++;
  }
  return cnt ? (int)(acc / cnt) : 0;
}

static void flashSet(bool on, const char *why, int mean) {
  if (s_flash == on)
    return;
  s_flash = on;
  digitalWrite(FLASH_PIN, on ? HIGH : LOW);
  s_probing = 0;
  if (on)
    s_probeIn = FLASH_AUTO_PROBE_EVERY;
  if (mean < 0)
    Serial.printf("[QR] flash %s (%s)\n", on ? "ON" : "OFF", why);
  else
    Serial.printf("[QR] flash %s (%s mean=%d)\n", on ? "ON" : "OFF", why,
                  mean);
}

// Call once per captured frame with that frame's mean. The frame was
// captured under the pre-call LED state; a change applies to next frames.
static void flashAutoUpdate(int mean) {
  if (!s_flashAuto)
    return;
  if (!s_flash) {
    if (s_probing > 0) {
      if (--s_probing == 0) {
        // Verdict on the AEC-settled probe frame.
        if (mean >= FLASH_AUTO_DARK_MEAN)
          s_probeIn = FLASH_AUTO_PROBE_EVERY; // ambient suffices, stay off
        else
          flashSet(true, "auto probe dark", mean);
      }
      return;
    }
    if (mean < FLASH_AUTO_DARK_MEAN)
      flashSet(true, "auto dark", mean);
  } else if (--s_probeIn <= 0) {
    flashSet(false, "auto probe", mean); // meter ambient unlit
    s_probing = FLASH_AUTO_PROBE_N;
  }
}

// ---------- error-path self-test (no camera needed) ----------
// Deterministic checks of the wrapper's argument guards + backend smoke:
// flat-gray 64x64 must decode to -1 (no QR found) with zero candidates.
static uint8_t s_blank[64 * 64];

static bool selfTest() {
  bool ok = true;
  qr_result_t res = {};
  int rc;

  // qrscan_free on the zero-init workspace (plus double-free) must be
  // safe and leave ws reusable. NOTE: no separate scratch workspace here:
  // qr_workspace_t is ~39KB and the ESP32CAM DRAM budget fits exactly one
  // (s_ws) + s_blank — a second static overflows dram0_0_seg at link time.
  qrscan_free(&s_ws);
  qrscan_free(&s_ws);
  Serial.println("[SELF] qrscan_free(zero-init) x2 survived");

  rc = qrscan_decode(NULL, &s_ws, &res);
  Serial.printf("[SELF] null img -> %d (%s) %s\n", rc, qrscan_strerror(rc),
                rc == -6 ? "OK" : "FAIL");
  ok &= (rc == -6);

  qr_image_t tiny = {s_blank, 10, 10};
  rc = qrscan_decode(&tiny, &s_ws, &res);
  Serial.printf("[SELF] tiny 10x10 -> %d (%s) %s\n", rc, qrscan_strerror(rc),
                rc == -6 ? "OK" : "FAIL");
  ok &= (rc == -6);

  memset(s_blank, 0x80, sizeof(s_blank));
  qr_image_t blank = {s_blank, 64, 64};
  int count = -1;
  qr_candidate_t cands[QRSCAN_MAX_CANDS] = {};
  rc = qrscan_decode_verbose(&blank, &s_ws, &res, &count, cands,
                             QRSCAN_MAX_CANDS);
  Serial.printf("[SELF] flat 64x64 -> rc=%d (%s) count=%d %s\n", rc,
                qrscan_strerror(rc), count,
                (rc == -1 && count == 0) ? "OK" : "FAIL");
  ok &= (rc == -1 && count == 0);

  // strerror covers every code incl. unknown; free(NULL-quirc) must be safe.
  Serial.printf("[SELF] strerror: 0=%s -1=%s -2=%s -3=%s -4=%s -5=%s -6=%s "
                "99=%s\n",
                qrscan_strerror(0), qrscan_strerror(-1), qrscan_strerror(-2),
                qrscan_strerror(-3), qrscan_strerror(-4), qrscan_strerror(-5),
                qrscan_strerror(-6), qrscan_strerror(99));
  // Release the handle the flat test allocated; the camera loop below
  // must lazily re-create it (exercises the free→realloc cycle).
  qrscan_free(&s_ws);

  Serial.printf("[SELF] %s\n", ok ? "ALL SELF-TESTS PASSED" : "*** SELF-TEST FAILED ***");
  return ok;
}

// ---------- full-res frame dump (F key) ----------
// Prints one VGA grayscale frame as hex lines between
// [QR] pgm begin W H / [QR] pgm end markers. Host side: collect the hex
// lines -> P5 PGM. Shows exactly the pixels quirc decodes.
// 640x480 = 4800 lines of 128 hex chars (~55 s @115200); yields regularly
// so the task WDT stays fed during the dump.
static void dumpFramePgm() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[QR] capture failed");
    return;
  }
  if (fb->format != PIXFORMAT_GRAYSCALE || fb->width <= 0 || fb->height <= 0) {
    Serial.printf("[QR] bad frame %dx%d fmt %d\n", fb->width, fb->height,
                  (int)fb->format);
    esp_camera_fb_return(fb);
    return;
  }
  static const char kHex[] = "0123456789abcdef";
  int mean = frameMean(fb->buf, (size_t)fb->width * (size_t)fb->height);
  Serial.printf("[QR] pgm begin %d %d flash=%d mean=%d\n", fb->width, fb->height,
                s_flash ? 1 : 0, mean);
  char line[129];
  line[128] = '\0';
  size_t n = (size_t)fb->width * (size_t)fb->height;
  size_t ln = 0;
  for (size_t i = 0; i < n; i += 64, ln++) {
    size_t chunk = (n - i) < 64 ? (n - i) : 64;
    for (size_t k = 0; k < 64; k++) {
      uint8_t v = (k < chunk) ? fb->buf[i + k] : 0;
      line[2 * k] = kHex[v >> 4];
      line[2 * k + 1] = kHex[v & 15];
    }
    Serial.println(line);
    if ((ln & 31) == 0)
      delay(1); // feed the task WDT during the ~55 s dump
  }
  Serial.println("[QR] pgm end");
  esp_camera_fb_return(fb);
}

static void scanOnce(bool dumpAscii) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[QR] capture failed");
    return;
  }
  bool grayOk =
      (fb->format == PIXFORMAT_GRAYSCALE && fb->width > 0 && fb->height > 0);
  if (!grayOk) {
    Serial.printf("[QR] bad frame %dx%d fmt %d\n", fb->width, fb->height,
                  (int)fb->format);
    esp_camera_fb_return(fb);
    return;
  }

  qr_image_t img = {fb->buf, (int)fb->width, (int)fb->height};
  qr_result_t res = {};
  int count = 0;
  int mean = frameMean(fb->buf, (size_t)fb->width * (size_t)fb->height);
  qr_candidate_t cands[QRSCAN_MAX_CANDS] = {};
  int rc = qrscan_decode_verbose(&img, &s_ws, &res, &count, cands,
                                 QRSCAN_MAX_CANDS);
  s_frameN++;
  int full_n = count / 100, half_n = count % 100;
  Serial.printf(
      "[QR] f%lu %dx%d full=%d half=%d rc=%d (%s) flash=%d mean=%d\n", s_frameN,
      fb->width, fb->height, full_n, half_n, rc, qrscan_strerror(rc),
      s_flash ? 1 : 0, mean);
  flashAutoUpdate(mean); // LED change (if any) applies to next frames
  int nshow = full_n + half_n;
  if (nshow > QRSCAN_MAX_CANDS)
    nshow = QRSCAN_MAX_CANDS;
  for (int i = 0; i < nshow; i++) {
    Serial.printf("[QR]   cand%d size=%d err=%d plen=%d "
                  "corners=(%d,%d)(%d,%d)(%d,%d)(%d,%d)\n",
                  i, cands[i].size, cands[i].decode_err, cands[i].payload_len,
                  cands[i].corners[0][0], cands[i].corners[0][1],
                  cands[i].corners[1][0], cands[i].corners[1][1],
                  cands[i].corners[2][0], cands[i].corners[2][1],
                  cands[i].corners[3][0], cands[i].corners[3][1]);
  }
  if (rc == 0) {
    Serial.printf("[QR] DECODED v%d ecc=%c mask=%d dim=%d len=%d data=\"%s\"\n",
                  res.version, res.ecc, res.mask, res.dim, res.len, res.data);
    digitalWrite(LED_PIN, LOW); // blink on success
    delay(80);
    digitalWrite(LED_PIN, HIGH);
  } else if (full_n == 0 && (s_frameN % 5) == 0) {
    Serial.println("[QR] hint: no QR found — hold code 15-25cm in front of "
                   "lens, fill 1/4+ of frame, flat, lit");
  }
  if (dumpAscii) {
    Serial.println("[QR] ascii view (76x30, #=dark):");
    qrscan_dump_image_ascii(&img, 128);
  }
  esp_camera_fb_return(fb);
}

static void handleUsb() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      s_line.trim();
      if (s_line.length() == 1) {
        char k = s_line.charAt(0);
        if (k == 'C' || k == 'c') {
          s_continuous = !s_continuous;
          Serial.printf("[QR] continuous scan %s\n",
                        s_continuous ? "ON" : "OFF");
        } else if (k == 'S' || k == 's') {
          scanOnce(false);
        } else if (k == 'D' || k == 'd') {
          scanOnce(true);
        } else if (k == 'F' || k == 'f') {
          dumpFramePgm();
        } else if (k == 'L' || k == 'l') {
          s_flashAuto = false; // explicit manual control wins over auto
          flashSet(!s_flash, "manual", -1);
        } else if (k == 'A' || k == 'a') {
          s_flashAuto = !s_flashAuto;
          s_probing = 0;
          s_probeIn = FLASH_AUTO_PROBE_EVERY;
          Serial.printf("[QR] flash auto %s\n", s_flashAuto ? "ON" : "OFF");
        } else if (k == 'V' || k == 'v') {
          s_vflip = !s_vflip;
          sensor_t *sens = esp_camera_sensor_get();
          int rv = -9;
          if (sens && sens->set_vflip)
            rv = sens->set_vflip(sens, s_vflip);
          Serial.printf("[QR] vflip %d hmirror %d (vflip rc=%d)\n", s_vflip,
                        s_hmirror, rv);
        } else if (k == 'M' || k == 'm') {
          s_hmirror = !s_hmirror;
          sensor_t *sens = esp_camera_sensor_get();
          int rm = -9;
          if (sens && sens->set_hmirror)
            rm = sens->set_hmirror(sens, s_hmirror);
          Serial.printf("[QR] vflip %d hmirror %d (hmirror rc=%d)\n", s_vflip,
                        s_hmirror, rm);
        } else if (k == 'H' || k == 'h' || k == '?') {
          Serial.println("keys: C continuous on/off, S single snapshot, "
                         "D snapshot + ascii, F full-frame pgm dump, "
                         "L flash manual (auto off), A flash auto, V vflip, "
                         "M hmirror, H help");
        } else {
          Serial.println("keys: C continuous on/off, S single snapshot, "
                         "D snapshot + ascii, F full-frame pgm dump, "
                         "L flash manual (auto off), A flash auto, V vflip, "
                         "M hmirror, H help");
        }
      } else if (s_line.length() > 0) {
        Serial.println("keys: C continuous on/off, S single snapshot, "
                       "D snapshot + ascii, F full-frame pgm dump, "
                       "L flash manual (auto off), A flash auto, V vflip, "
                       "M hmirror, H help");
      }
      s_line = "";
    } else {
      s_line += c;
      if (s_line.length() > 16)
        s_line = "";
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED off (active low)
  pinMode(FLASH_PIN, OUTPUT);
  digitalWrite(FLASH_PIN, s_flash ? HIGH : LOW); // flash on at boot

  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== ESP32CAM QR SCAN TEST ===");
  Serial.println("Reuses src/qrscan.c + include/qrscan.h over src/quirc/.");
  Serial.println("Keys: C continuous, S snapshot, D snapshot+ascii, "
                   "F pgm dump, L flash manual, A flash auto, V vflip, "
                   "M hmirror, H help.");

  memset(&s_ws, 0, sizeof(s_ws));
  bool selfOk = selfTest();
  if (!selfOk)
    Serial.println("*** continuing anyway: live camera may still decode ***");

  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer = LEDC_TIMER_0;
  cfg.pin_d0 = CAM_PIN_Y2;
  cfg.pin_d1 = CAM_PIN_Y3;
  cfg.pin_d2 = CAM_PIN_Y4;
  cfg.pin_d3 = CAM_PIN_Y5;
  cfg.pin_d4 = CAM_PIN_Y6;
  cfg.pin_d5 = CAM_PIN_Y7;
  cfg.pin_d6 = CAM_PIN_Y8;
  cfg.pin_d7 = CAM_PIN_Y9;
  cfg.pin_xclk = CAM_PIN_XCLK;
  cfg.pin_pclk = CAM_PIN_PCLK;
  cfg.pin_vsync = CAM_PIN_VSYNC;
  cfg.pin_href = CAM_PIN_HREF;
  cfg.pin_sccb_sda = CAM_PIN_SIOD;
  cfg.pin_sccb_scl = CAM_PIN_SIOC;
  cfg.pin_pwdn = CAM_PIN_PWDN;
  cfg.pin_reset = CAM_PIN_RESET;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_GRAYSCALE;
  cfg.frame_size = FRAMESIZE_VGA;
  cfg.jpeg_quality = 0; // unused for grayscale
  cfg.fb_count = 1;
  cfg.fb_location = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println("*** camera init failed: self-test only. ***");
    s_continuous = false;
    return;
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_brightness(s, 0);
    s->set_contrast(s, 2); // same QR-tuned DSP as the provisioning scan
    s->set_ae_level(s, 0);
    // This rig's module is mounted upside-down: rotate 180 degrees.
    // (Decode is rotation-invariant; this is for human viewing + logs.)
    // Return codes are logged: a -1 here means that axis is unsupported and
    // the stream ends up mirrored, not rotated.
    int rv = -9, rm = -9;
    if (s->set_vflip)
      rv = s->set_vflip(s, s_vflip);
    if (s->set_hmirror)
      rm = s->set_hmirror(s, s_hmirror);
    Serial.printf("Camera orientation: vflip=%d hmirror=%d (rc=%d/%d)\n",
                  s_vflip, s_hmirror, rv, rm);
  }
  Serial.printf("Camera VGA %dx%d gray ready. Continuous %s.\n", QR_W, QR_H,
                s_continuous ? "ON" : "OFF");
  Serial.println("===============================");
}

void loop() {
  handleUsb();
  if (s_continuous) {
    scanOnce(false);
    delay(SCAN_PAUSE_MS);
  } else {
    delay(50);
  }
}
