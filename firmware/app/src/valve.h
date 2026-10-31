#pragma once

// 3-relay valve actuator driver (spec docs/aipool-spec.md).
// Map: R1=MODE (OFF=FILTER, ON=BACKWASH), R2=BACKWASH drive, R3=FILTER drive.
// Motion: drives OFF -> set MODE -> 200ms settle -> pulse target drive for
// travel time -> drives OFF -> readback verify. Never R2+R3 together.
// NOTE: travel time + MODE polarity are commissioning assumptions — flip
// VALVE_MODE_ON_IS_BACKWASH or VALVE_TRAVEL_MS here after first dry run.

#include <stdint.h>

#define VALVE_RELAY_MODE 1
#define VALVE_RELAY_BACKWASH_DRIVE 2
#define VALVE_RELAY_FILTER_DRIVE 3

#define VALVE_MODE_SETTLE_MS 200
#define VALVE_TRAVEL_MS 5000
#define VALVE_MODE_ON_IS_BACKWASH 1

// Fail-safe: all drives OFF. Call once at boot before anything else.
bool valveInit();

// Blocking motion (~200ms + travel). Returns true on echo+readback OK.
// Must only be called after pump_stopped_ok + 15s pre-hold (main.cpp).
bool valveGotoBackwash();
bool valveGotoFilter();

// Readback: bitmap of relays 1..8 via 0x01. True on valid reply.
bool valveReadback(uint8_t &bitmapOut);
const char *valveLastError();
