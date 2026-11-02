#pragma once

// WiFi provisioning: camera-reads-QR -> NVS -> connect (retry scan forever).
// QR payload: WIFI:T:WPA;S:<ssid>;P:<pass>;; (spec docs/aipool-spec.md).
// Decode via qrscan.h (quirc backend); camera is the vision VGA grayscale
// stream — call visionInit() once before provScanQr(), then provScanQr()
// borrows frames via esp_camera_fb_get() until the timeout expires.

#include <stddef.h>
#include <stdint.h>

#define PROV_SCAN_TIMEOUT_MS 15000

// Parse WIFI:... payload. Returns true with ssid/pass filled (pass may be "").
bool provParseWifi(const char *payload, char *ssidOut, size_t ssidCap,
                   char *passOut, size_t passCap);

// Hold the QR in front of the camera for up to timeoutMs; LED blinks.
// Returns true with the raw decoded payload in outPayload.
bool provScanQr(uint32_t timeoutMs, char *outPayload, size_t outCap);
