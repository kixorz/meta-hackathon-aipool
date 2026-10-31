// Vision: ESP32CAM OV2640 VGA grayscale -> gauge.c.
// Spec: 640x480 gray (307kpx < 400kpx guard), scratch in PSRAM, ~1Hz,
// defaults zero 131.0 / max 46.5 deg, 0-60psi. QR provisioning TODO.

#include "vision.h"

#include <Arduino.h>
#include <esp_camera.h>

#include "gauge.h"

// AI Thinker ESP32-CAM pins.
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

#define VISION_W 640
#define VISION_H 480

static uint8_t *s_scratch = nullptr;
static size_t s_scratchLen = 0;
static uint8_t *s_frame = nullptr; // VGA gray copy for gauge + /gauge
static gauge_tracker_t s_tracker;
static gauge_calib_t s_calib = {131.0f, 46.5f, 0.0f, 60.0f, 0};
static VisionReading s_latest;
static char s_err[64] = {0};
static bool s_ready = false;

const char *visionLastError() { return s_err; }
VisionReading visionLatest() { return s_latest; }
const uint8_t *visionFrame() { return s_frame; }

bool visionInit() {
  gauge_tracker_init(&s_tracker);

  s_scratchLen = gauge_scratch_size(VISION_W, VISION_H);
  s_scratch = (uint8_t *)ps_malloc(s_scratchLen);
  s_frame = (uint8_t *)ps_malloc((size_t)VISION_W * VISION_H);
  if (!s_scratch || !s_frame) {
    snprintf(s_err, sizeof(s_err), "psram alloc failed");
    return false;
  }

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
    snprintf(s_err, sizeof(s_err), "camera init failed");
    return false;
  }
  // Stable exposure for dial: disable auto artifacts that break needle vote.
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_awb_gain(s, 1);
  }
  s_ready = true;
  return true;
}

bool visionPoll() {
  if (!s_ready) {
    snprintf(s_err, sizeof(s_err), "camera not ready");
    return false;
  }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    snprintf(s_err, sizeof(s_err), "capture failed");
    return false;
  }
  bool ok = false;
  if (fb->width == VISION_W && fb->height == VISION_H &&
      fb->format == PIXFORMAT_GRAYSCALE) {
    memcpy(s_frame, fb->buf, (size_t)VISION_W * VISION_H);
    gauge_hint_t hint = {(int)s_tracker.init, s_tracker.cx, s_tracker.cy,
                         s_tracker.R};
    gauge_result_t r = {};
    gauge_status_t st =
        gauge_read_gray(s_frame, VISION_W, VISION_H, &s_calib, &hint, s_scratch,
                        s_scratchLen, &r);
    if (st == GAUGE_OK) {
      gauge_stab_t stab = {};
      if (gauge_tracker_update(&s_tracker, &r, &s_calib, hint.enable,
                               &stab) == GAUGE_OK) {
        s_latest.ok = true;
        s_latest.psi = stab.median_value;
        s_latest.angleDeg = stab.stab_angle;
        s_latest.confSigma = r.conf_sigma;
        s_latest.support = r.support;
        s_latest.blur = r.blur;
        s_latest.err = "none";
        ok = true;
      } else {
        snprintf(s_err, sizeof(s_err), "tracker failed");
      }
    } else if (st == GAUGE_ERR_NODIAL) {
      snprintf(s_err, sizeof(s_err), "no dial");
      gauge_tracker_reset(&s_tracker);
    } else {
      snprintf(s_err, sizeof(s_err), "read err %d", (int)st);
    }
  } else {
    snprintf(s_err, sizeof(s_err), "bad frame %dx%d fmt %d", fb->width,
             fb->height, (int)fb->format);
  }
  esp_camera_fb_return(fb);
  if (!ok)
    s_latest.ok = false;
  return ok;
}
