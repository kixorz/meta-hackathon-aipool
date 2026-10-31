#pragma once

// Vision service: OV2640 VGA grayscale -> gauge.c pressure reading.
// Owns camera, PSRAM scratch + frame copy, tracker + calibration.
// Consumers poll visionLatest(); /gauge serves the frame as BMP.

#include <stddef.h>
#include <stdint.h>

struct VisionReading {
  bool ok = false;
  float psi = 0.0f;
  float angleDeg = 0.0f;
  float confSigma = 0.0f;
  float support = 0.0f;
  float blur = 0.0f;
  const char *err = "none";
};

bool visionInit();          // camera + PSRAM + tracker; false = no camera
bool visionPoll();          // capture + read + track; fills latest (~1Hz call)
VisionReading visionLatest();

// Latest VGA gray frame (640x480) for /gauge BMP. Null if none yet.
const uint8_t *visionFrame();
const char *visionLastError();
