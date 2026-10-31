#pragma once

// WiFi provisioning: camera-reads-QR -> NVS -> connect, AP fallback.
// QR payload: WIFI:T:WPA;S:<ssid>;P:<pass>;; (spec docs/aipool-spec.md).
// Camera is owned by vision.cpp (VGA grayscale); scanning reuses its frames.

#include <stddef.h>
#include <stdint.h>

#define PROV_SCAN_TIMEOUT_MS 15000

// Parse WIFI:... payload. Returns true with ssid/pass filled (pass may be "").
bool provParseWifi(const char *payload, char *ssidOut, size_t ssidCap,
                   char *passOut, size_t passCap);

// Hold the QR in front of the camera for up to timeoutMs; LED blinks.
// Returns true with the raw decoded payload in outPayload.
bool provScanQr(uint32_t timeoutMs, char *outPayload, size_t outCap);
