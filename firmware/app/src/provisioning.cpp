// Provisioning: decode WIFI QR from the already-running VGA camera.
// Decode goes through qrscan_decode (quirc vendored under src/quirc/, MIT).
// No external QR dependency; do not call quirc_* directly here.

#include "provisioning.h"
#include "qrscan.h"

#include <Arduino.h>
#include <esp_camera.h>
#include <esp_task_wdt.h>

#include <string.h>

#define LED_PIN 33
#define PROV_FLASH_PIN 4 // ESP32CAM white flash LED: on for the whole scan

static void provFeedWdt() {
  yield();
  // Reset task WDT when enabled (setup() blocks >5s during 15s QR scan).
  esp_task_wdt_reset();
}

// Laplacian variance over the candidate bbox: <~50 = blurry/defocused,
// >~150 = reasonably sharp. Stride 3 in x/y (1:9 of pixels) for speed.
// Guides lens-focus vs lighting diagnosis without needing /gauge
// (HTTP isn't up yet).
static float provRoiSharp(const uint8_t *gray, int w, int h,
                          const qr_candidate_t *c) {
  int x0 = c->corners[0][0], x1 = c->corners[0][0];
  int y0 = c->corners[0][1], y1 = c->corners[0][1];
  for (int k = 1; k < 4; k++) {
    if (c->corners[k][0] < x0) x0 = c->corners[k][0];
    if (c->corners[k][0] > x1) x1 = c->corners[k][0];
    if (c->corners[k][1] < y0) y0 = c->corners[k][1];
    if (c->corners[k][1] > y1) y1 = c->corners[k][1];
  }
  if (x0 < 1) x0 = 1;
  if (y0 < 1) y0 = 1;
  if (x1 >= w - 1) x1 = w - 2;
  if (y1 >= h - 1) y1 = h - 2;
  if (x1 <= x0 || y1 <= y0) return 0.0f;
  double m = 0.0, m2 = 0.0;
  long n = 0;
  for (int y = y0; y <= y1; y += 3) {
    for (int x = x0; x <= x1; x += 3) {
      int c0 = gray[(size_t)y * w + x];
      int l = c0 * 4 - gray[(size_t)y * w + x - 1] -
              gray[(size_t)y * w + x + 1] - gray[(size_t)(y - 1) * w + x] -
              gray[(size_t)(y + 1) * w + x];
      m += l;
      m2 += (double)l * l;
      n++;
    }
  }
  if (!n) return 0.0f;
  return (float)(m2 / n - (m / n) * (m / n));
}

static void unescapeInto(const char *src, char *dst, size_t cap) {
  size_t n = 0;
  for (const char *p = src; *p && n + 1 < cap; p++) {
    if (*p == '\\' && p[1]) {
      p++;
      dst[n++] = *p;
    } else if (*p == ';') {
      break; // end of field (unescaped ;)
    } else {
      dst[n++] = *p;
    }
  }
  dst[n] = '\0';
}

// Find ";KEY:" or the first field right after "WIFI:" and return pointer
// to value, or null.
static const char *findField(const char *payload, const char *key) {
  size_t klen = strlen(key);
  const char *firstField = payload + 5; // caller verified "WIFI:" prefix
  for (const char *p = payload; *p; p++) {
    bool atStart =
        (p == payload) || (p == firstField) || (p[-1] == ';');
    if (atStart && strncmp(p, key, klen) == 0 && p[klen] == ':') {
      return p + klen + 1;
    }
    // Skip escaped chars so "\;" isn't treated as separator.
    if (*p == '\\' && p[1])
      p++;
  }
  return nullptr;
}

bool provParseWifi(const char *payload, char *ssidOut, size_t ssidCap,
                   char *passOut, size_t passCap) {
  if (!payload || !ssidOut || ssidCap == 0 || !passOut || passCap == 0)
    return false;
  ssidOut[0] = '\0';
  passOut[0] = '\0';
  if (strncmp(payload, "WIFI:", 5) != 0)
    return false;
  const char *s = findField(payload, "S");
  if (!s || *s == '\0' || *s == ';')
    return false; // SSID required
  unescapeInto(s, ssidOut, ssidCap);
  if (ssidOut[0] == '\0')
    return false;
  const char *p = findField(payload, "P");
  if (p)
    unescapeInto(p, passOut, passCap);
  return true;
}

bool provScanQr(uint32_t timeoutMs, char *outPayload, size_t outCap) {
  if (!outPayload || outCap == 0)
    return false;
  // qr_workspace_t is ~39KB — MUST be heap/static, never stack (8KB).
  qr_workspace_t *ws =
      (qr_workspace_t *)ps_malloc(sizeof(qr_workspace_t));
  if (!ws)
    ws = (qr_workspace_t *)malloc(sizeof(qr_workspace_t));
  if (!ws)
    return false;
  memset(ws, 0, sizeof(*ws));
  // qrscan_decode owns the quirc handle inside ws (lazy alloc + resize);
  // quirc_code/data scratch lives in static storage inside qrscan.c.
  // Reused across frames for the whole scan.
  // QR-tuned capture: auto exposure/gain on + slight brightness/contrast
  // lift. VGA grayscale from visionInit is kept (no re-init); we only
  // nudge sensor DSP and restore defaults afterwards for gauge stability.
  sensor_t *sens = esp_camera_sensor_get();
  if (sens) {
    if (sens->set_gain_ctrl)
      sens->set_gain_ctrl(sens, 1);
    if (sens->set_exposure_ctrl)
      sens->set_exposure_ctrl(sens, 1);
    // Keep exposure neutral: brightness lift washes blacks toward quirc's
    // threshold. Contrast only, AE level 0. Sharpness/denoise left at
    // sensor defaults: max sharpness + denoise-off flooded detection
    // with noise edges (full=1 stable -> full=0) on this OV2640.
    if (sens->set_brightness)
      sens->set_brightness(sens, 0);
    if (sens->set_contrast)
      sens->set_contrast(sens, 2);
    if (sens->set_ae_level)
      sens->set_ae_level(sens, 0);
  }
  // Flash on for the whole scan: the setup scene is usually dark and the
  // QR needs light (bench-proven: unlit mean ~6, lit ~165). Restored off
  // with the DSP below; the next scan re-enables it.
  pinMode(PROV_FLASH_PIN, OUTPUT);
  digitalWrite(PROV_FLASH_PIN, HIGH);
  bool found = false;
  unsigned long t0 = millis();
  bool led = false;
  unsigned long lastBlink = 0;
  unsigned frameN = 0;
  int prevCorners[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
  int havePrev = 0;

  while (millis() - t0 < timeoutMs && !found) {
    provFeedWdt();
    if (millis() - lastBlink > 200) {
      lastBlink = millis();
      led = !led;
      digitalWrite(LED_PIN, led ? LOW : HIGH); // active LOW
    }
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      delay(200);
      continue;
    }
    if (fb->format == PIXFORMAT_GRAYSCALE && fb->width > 0 &&
        fb->height > 0) {
      qr_image_t img = {fb->buf, (int)fb->width, (int)fb->height};
      qr_result_t res = {};
      int count = 0;
      qr_candidate_t cands[QRSCAN_MAX_CANDS] = {};
      provFeedWdt();
      int rc = qrscan_decode_verbose(&img, ws, &res, &count, cands,
                                     QRSCAN_MAX_CANDS);
      provFeedWdt();
      frameN++;
      // Every frame while debugging the visible-but-unreadable code:
      // count + per-candidate geometry tells stable-QR vs noise;
      // sharp separates blur (focus/distance) from contrast (lighting).
      int full_n = count / 100, half_n = count % 100;
      float sharp =
          (full_n > 0)
              ? provRoiSharp(fb->buf, fb->width, fb->height, &cands[0])
              : 0.0f;
      int jit = 0, gridW = 0;
      if (full_n > 0) {
        int dx = cands[0].corners[0][0] - cands[0].corners[1][0];
        int dy = cands[0].corners[0][1] - cands[0].corners[1][1];
        gridW = (int)(sqrtf((float)(dx * dx + dy * dy)) + 0.5f);
        if (havePrev) {
          for (int k = 0; k < 4; k++) {
            int jx = cands[0].corners[k][0] - prevCorners[k][0];
            int jy = cands[0].corners[k][1] - prevCorners[k][1];
            int j = (jx < 0 ? -jx : jx) + (jy < 0 ? -jy : jy);
            if (j > jit)
              jit = j;
          }
        }
        for (int k = 0; k < 4; k++) {
          prevCorners[k][0] = cands[0].corners[k][0];
          prevCorners[k][1] = cands[0].corners[k][1];
        }
        havePrev = 1;
      }
      Serial.printf(
          "[prov] qr f%u %dx%d full=%d half=%d rc=%d (%s) sharp=%.0f "
          "grid=%dpx jit=%dpx\n",
          frameN, fb->width, fb->height, full_n, half_n, rc,
          qrscan_strerror(rc), sharp, gridW, jit);
      // Live placement coaching: the user is holding the code now.
      if (full_n == 0 && (frameN % 5) == 0) {
        Serial.println(
            "[prov] hint: no QR found - hold code 15-25cm in front of lens, "
            "fill 1/4+ of frame, flat, lit");
      } else if (full_n > 0 && rc != 0 && (frameN % 5) == 0) {
        if (jit > 6)
          Serial.println("[prov] hint: code is MOVING - brace hands, hold "
                         "still 2-3s");
        else if (gridW > 0 && gridW < 150)
          Serial.println("[prov] hint: code too SMALL - move closer / print "
                         "bigger (aim 250-350px wide)");
        else if (gridW > 380)
          Serial.println("[prov] hint: code too BIG/crop - pull back, keep "
                         "all 3 squares + white margin in view");
        else
          Serial.println("[prov] hint: seen but unreadable - flatten paper, "
                         "kill glare, white backing, steady, centered");
      }
      int nshow = full_n + half_n;
      if (nshow > QRSCAN_MAX_CANDS) nshow = QRSCAN_MAX_CANDS;
      for (int i = 0; i < nshow; i++) {
        Serial.printf(
            "[prov]   cand%d size=%d err=%d plen=%d "
            "corners=(%d,%d)(%d,%d)(%d,%d)(%d,%d)\n",
            i, cands[i].size, cands[i].decode_err, cands[i].payload_len,
            cands[i].corners[0][0], cands[i].corners[0][1],
            cands[i].corners[1][0], cands[i].corners[1][1],
            cands[i].corners[2][0], cands[i].corners[2][1],
            cands[i].corners[3][0], cands[i].corners[3][1]);
      }
      // When quirc sees a grid but can't decode, dump a thumbnail so we
      // can tell cropped/glare/blur from a distance without /gauge
      // (HTTP isn't up yet — setup blocks before server start).
      if (count / 100 > 0 && rc != 0 && (frameN % 3) == 1) {
        Serial.println("[prov] ascii view (76x30, #=dark):");
        const int step_y = fb->height / 30 + 1, step_x = fb->width / 76 + 1;
        for (int y = 0; y < fb->height; y += step_y) {
          for (int x = 0; x < fb->width; x += step_x)
            Serial.print(fb->buf[(size_t)y * fb->width + x] < 128 ? '#' : '.');
          Serial.println();
          if ((y / step_y) % 10 == 9)
            provFeedWdt();
        }
      }
      // One full-res ROI hex dump per boot for host-side forensics: the
      // exact pixels quirc chokes on. Decode the hex back to PGM on the
      // host and run the vendored quirc harness against it.
      static bool roiDumped = false;
      if (!roiDumped && count / 100 > 0 && rc != 0) {
        roiDumped = true;
        int x0 = cands[0].corners[0][0], x1 = x0;
        int y0 = cands[0].corners[0][1], y1 = y0;
        for (int k = 1; k < 4; k++) {
          if (cands[0].corners[k][0] < x0) x0 = cands[0].corners[k][0];
          if (cands[0].corners[k][0] > x1) x1 = cands[0].corners[k][0];
          if (cands[0].corners[k][1] < y0) y0 = cands[0].corners[k][1];
          if (cands[0].corners[k][1] > y1) y1 = cands[0].corners[k][1];
        }
        x0 -= 12;
        y0 -= 12;
        x1 += 12;
        y1 += 12;
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x1 >= fb->width) x1 = fb->width - 1;
        if (y1 >= fb->height) y1 = fb->height - 1;
        int rw = x1 - x0 + 1, rh = y1 - y0 + 1;
        Serial.printf("[prov] roi x=%d y=%d w=%d h=%d stride=2 hex:\n", x0,
                      y0, rw, rh);
        static const char *hexd = "0123456789abcdef";
        // Stride 2 keeps the dump (~1/4 bytes) inside the scan budget while
        // preserving every module edge for host forensics.
        for (int y = y0; y <= y1; y += 2) {
          for (int x = x0; x <= x1; x += 2) {
            uint8_t v = fb->buf[(size_t)y * fb->width + x];
            Serial.print(hexd[v >> 4]);
            Serial.print(hexd[v & 15]);
          }
          Serial.println();
          if ((((y - y0) >> 1) & 15) == 15)
            provFeedWdt();
        }
        Serial.println("[prov] roi end");
      }
      if (rc == 0 && res.len > 0 && (size_t)res.len < outCap) {
        Serial.printf("[prov] DECODED v%d ecc=%c mask=%d dim=%d len=%d\n",
                      res.version, res.ecc, res.mask, res.dim, res.len);
        memcpy(outPayload, res.data, (size_t)res.len);
        outPayload[res.len] = '\0';
        found = true;
      }
    }
    esp_camera_fb_return(fb);
    if (!found) {
      provFeedWdt();
      delay(150);
    }
  }
  // Restore neutral DSP for the gauge path (visionPoll post-provisioning).
  if (sens) {
    if (sens->set_brightness)
      sens->set_brightness(sens, 0);
    if (sens->set_contrast)
      sens->set_contrast(sens, 0);
    if (sens->set_ae_level)
      sens->set_ae_level(sens, 0);
  }
  qrscan_free(ws);
  free(ws);
  digitalWrite(PROV_FLASH_PIN, LOW); // scan over: flash back off
  digitalWrite(LED_PIN, HIGH); // scan over: LED back to idle-off
  return found;
}
