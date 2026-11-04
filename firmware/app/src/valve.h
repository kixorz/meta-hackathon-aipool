#pragma once

// 3-relay valve actuator (bench truth, docs/aipool-spec.md).
// Positions are LATCHED relay states, held energized while selected:
//   FILTER:   R2 ON (R1,R3 OFF)             bitmap 0x02
//   BACKWASH: R1 ON + R3 ON (R2 OFF)        bitmap 0x05
// Moves: all OFF (break-before-make) -> latch target -> travel ~20 s ->
// verify latched bitmap. The actuator needs ~20 s end to end; commanding
// the next position earlier reverses it mid-travel.
// NOTE: an earlier pulse model (MODE + 5 s drive pulse, then all OFF)
// never moved this hardware: 5 s << travel, and the relay roles differ.
// Do not reintroduce pulsing without a bench run.

#include <stdint.h>

#include "relay_board.h"

#define VALVE_RELAY_R1 1
#define VALVE_RELAY_R2 2
#define VALVE_RELAY_R3 3

#define VALVE_BITMAP_FILTER (RELAY_BOARD_BIT(2))
#define VALVE_BITMAP_BACKWASH (RELAY_BOARD_BIT(1) | RELAY_BOARD_BIT(3))

#define VALVE_MODE_SETTLE_MS 200   // release gap between all-off and latch
#define VALVE_TRAVEL_MS 20000      // full-range travel time (bench-measured)

// Fail-safe: valve relays R1/R2/R3 OFF. Call once at boot before anything else.
bool valveInit();

// Blocking move (~200ms + travel). Holds the latched position while selected.
// Must only be called after pump_stopped_ok + 15s pre-hold (main.cpp).
bool valveGotoBackwash();
bool valveGotoFilter();

// Readback: bitmap of relays 1..8 via 0x01. True on valid reply.
bool valveReadback(uint8_t &bitmapOut);
const char *valveLastError();
