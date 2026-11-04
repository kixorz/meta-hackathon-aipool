// Valve actuator: 3 relays over shared RS485 Modbus board.
// Controls-safety contract: caller guarantees pump stopped + 15s pre-hold;
// this file guarantees break-before-make, latched position, travel time.

#include "valve.h"

#include <Arduino.h>

#include "relay_board.h"
#include "rs485_io.h"

static char s_err[64] = {0};

static bool writeRelay(uint8_t relay, bool on) {
  uint8_t tx[RELAY_BOARD_WRITE_REQ_LEN];
  uint8_t rx[16];
  size_t txLen =
      relay_board_build_write_coil(RELAY_BOARD_SLAVE_ADDR, relay, on, tx);
  if (txLen == 0) {
    snprintf(s_err, sizeof(s_err), "bad relay arg R%u", relay);
    return false;
  }
  size_t n = rs485Txn(tx, txLen, rx, sizeof(rx));
  if (!relay_board_is_write_echo(tx, rx, n)) {
    snprintf(s_err, sizeof(s_err), "R%u %s no echo (A/B? GND? 12V? slave 1)",
             relay, on ? "ON" : "OFF");
    return false;
  }
  return true;
}

static bool allDrivesOff() {
  // Fail-safe: valve relays R1/R2/R3 OFF (bench safe state; the test rig
  // ends here too). Position relays are re-latched on the next move.
  if (!writeRelay(VALVE_RELAY_R1, false))
    return false;
  if (!writeRelay(VALVE_RELAY_R2, false))
    return false;
  if (!writeRelay(VALVE_RELAY_R3, false))
    return false;
  return true;
}

bool valveInit() {
  s_err[0] = '\0';
  // Retry once: bus may still be settling after boot.
  if (allDrivesOff())
    return true;
  delay(200);
  return allDrivesOff();
}

bool valveReadback(uint8_t &bitmapOut) {
  uint8_t tx[RELAY_BOARD_READ_REQ_LEN];
  uint8_t rx[16];
  size_t txLen = relay_board_build_read_coils(RELAY_BOARD_SLAVE_ADDR, tx);
  if (txLen == 0)
    return false;
  size_t n = rs485Txn(tx, txLen, rx, sizeof(rx));
  return relay_board_parse_coils(rx, n, RELAY_BOARD_SLAVE_ADDR, &bitmapOut);
}

const char *valveLastError() { return s_err; }

static bool driveTo(bool backwash) {
  s_err[0] = '\0';
  uint8_t target =
      backwash ? VALVE_BITMAP_BACKWASH : VALVE_BITMAP_FILTER;
  // 1. Break-before-make: everything OFF first.
  if (!allDrivesOff())
    return false;
  delay(VALVE_MODE_SETTLE_MS);
  // 2. Latch the target position (held energized while selected).
  if (!writeRelay(VALVE_RELAY_R1,
                  (target & RELAY_BOARD_BIT(1)) != 0))
    return false;
  if (!writeRelay(VALVE_RELAY_R2,
                  (target & RELAY_BOARD_BIT(2)) != 0))
    return false;
  if (!writeRelay(VALVE_RELAY_R3,
                  (target & RELAY_BOARD_BIT(3)) != 0))
    return false;
  // 3. Travel time: the actuator needs ~20 s end to end.
  delay(VALVE_TRAVEL_MS);
  // 4. Verify latched: readback must equal the target bitmap (masked to
  // the valve relays; relays 4-8 belong to other circuits, if any).
  uint8_t bm = 0;
  if (!valveReadback(bm)) {
    snprintf(s_err, sizeof(s_err), "readback failed (A/B? GND? 12V?)");
    return false;
  }
  if ((bm & 0x07) != target) {
    snprintf(s_err, sizeof(s_err),
             "pos not latched bm=0x%02X want 0x%02X", bm, target);
    return false;
  }
  return true;
}

bool valveGotoBackwash() { return driveTo(true); }
bool valveGotoFilter() { return driveTo(false); }
