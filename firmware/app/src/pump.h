#pragma once

// Century VGreen pump driver over the shared RS485 bus.
// Wraps century_client_t (src/century.c) with bus-mutex transport.
// Policy: 60s set_demand_and_run refresh (volatile demand register),
// dropout detection, fail-safe stop. Numbers from docs/aipool-spec.md.

#include <stdint.h>
#include "century.h"

struct PumpReading {
  bool present = false; // any successful reply since boot
  bool ok = false;      // last poll fully OK
  century_status_t status = CENTURY_STATUS_UNKNOWN;
  uint16_t rpm = 0;
  bool rpmOk = false;
  uint16_t demand = 0;
  bool demandOk = false;
  uint16_t watts = 0;
  bool wattsOk = false;
  uint8_t faultCode = 0;
  const char *err = "none";
};

void pumpInit(uint16_t targetRpm);
bool pumpStop();
bool pumpRun(uint16_t rpm); // set_demand_and_run
bool pumpPoll(PumpReading &out);
bool pumpRefreshIfDue(uint16_t targetRpm); // 60s re-arm, call from loop
bool pumpStoppedOk();                      // true if confirmed stopped
const char *pumpLastError();
