// Valve actuator: 3 relays over shared RS485 Modbus board.
// Controls-safety contract: caller guarantees pump stopped + 15s pre-hold;
// this file guarantees break-before-make, echo verify, fail-safe OFF.

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
  // MODE is a selector, not a drive — but parking it at FILTER (OFF) is the
  // documented fail-safe. Drives R2/R3 must never be on together.
  if (!writeRelay(VALVE_RELAY_BACKWASH_DRIVE, false))
    return false;
  if (!writeRelay(VALVE_RELAY_FILTER_DRIVE, false))
    return false;
  if (!writeRelay(VALVE_RELAY_MODE, false))
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
#if VALVE_MODE_ON_IS_BACKWASH
  bool modeOn = backwash;
#else
  bool modeOn = !backwash;
#endif
  uint8_t driveRelay = backwash ? VALVE_RELAY_BACKWASH_DRIVE
                                : VALVE_RELAY_FILTER_DRIVE;

  // 1. Break-before-make: everything OFF first.
  if (!writeRelay(VALVE_RELAY_BACKWASH_DRIVE, false))
    return false;
  if (!writeRelay(VALVE_RELAY_FILTER_DRIVE, false))
    return false;
  // 2. Select direction, let contactor settle.
  if (!writeRelay(VALVE_RELAY_MODE, modeOn))
    return false;
  delay(VALVE_MODE_SETTLE_MS);
  // 3. Pulse the target drive for travel time.
  if (!writeRelay(driveRelay, true))
    return false;
  delay(VALVE_TRAVEL_MS);
  if (!writeRelay(driveRelay, false))
    return false;
  // 4. Verify parked: no drive energized.
  uint8_t bm = 0;
  if (!valveReadback(bm)) {
    snprintf(s_err, sizeof(s_err), "readback failed (A/B? GND? 12V?)");
    return false;
  }
  if (relay_board_get(bm, VALVE_RELAY_BACKWASH_DRIVE) ||
      relay_board_get(bm, VALVE_RELAY_FILTER_DRIVE)) {
    snprintf(s_err, sizeof(s_err), "drive stuck ON bm=0x%02X", bm);
    return false;
  }
  return true;
}

bool valveGotoBackwash() { return driveTo(true); }
bool valveGotoFilter() { return driveTo(false); }
