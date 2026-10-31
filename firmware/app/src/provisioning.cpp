// Provisioning: decode WIFI QR from the already-running VGA camera.
// quirc is vendored under src/quirc/ (MIT). No external QR dependency.

#include "provisioning.h"

#include <Arduino.h>
#include <esp_camera.h>

#include "quirc/quirc.h"

#define LED_PIN 33

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
  struct quirc *qr = quirc_new();
  if (!qr)
    return false;
  // NB: quirc_code (~4KB bitmap) + quirc_data (~4KB payload) must NOT live
  // on loopTask's stack (8KB) — heap them (PSRAM preferred, else DRAM).
  struct quirc_code *code = (struct quirc_code *)ps_malloc(sizeof(*code));
  struct quirc_data *data = (struct quirc_data *)ps_malloc(sizeof(*data));
  if (!code || !data) {
    free(code);
    free(data);
    quirc_destroy(qr);
    return false;
  }
  bool found = false;
  unsigned long t0 = millis();
  bool led = false;
  unsigned long lastBlink = 0;

  while (millis() - t0 < timeoutMs && !found) {
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
      if (quirc_resize(qr, fb->width, fb->height) == 0) {
        uint8_t *img = quirc_begin(qr, nullptr, nullptr);
        if (img) {
          memcpy(img, fb->buf, (size_t)fb->width * fb->height);
          quirc_end(qr);
          if (quirc_count(qr) > 0) {
            quirc_decode_error_t err;
            quirc_extract(qr, 0, code);
            err = quirc_decode(code, data);
            if (err == QUIRC_SUCCESS ||
                err == QUIRC_ERROR_DATA_ECC ||
                err == QUIRC_ERROR_DATA_UNDERFLOW ||
                err == QUIRC_ERROR_DATA_OVERFLOW) {
              // ECC/underflow/overflow may still yield a usable payload.
              if (data->payload_len > 0 &&
                  (size_t)data->payload_len < outCap) {
                memcpy(outPayload, data->payload,
                       (size_t)data->payload_len);
                outPayload[data->payload_len] = '\0';
                found = true;
              }
            }
          }
        }
      }
    }
    esp_camera_fb_return(fb);
    if (!found)
      delay(150);
  }
  quirc_destroy(qr);
  free(code);
  free(data);
  digitalWrite(LED_PIN, HIGH); // scan over: LED back to idle-off
  return found;
}
