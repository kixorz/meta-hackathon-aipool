#pragma once

// Shared runtime state polled by GET /api/state @1Hz.
// Single-writer per field in later phases; for Phase 0 skeleton all
// fields are updated from loop() stubs. Keep this struct JSON-shaped
// to match docs/aipool-spec.md.

#include <stdint.h>

enum class PoolMode : uint8_t {
  FILTER = 0,
  BACKWASH = 1,
  MOVING = 2, // transient: valve in transit (15s pre-hold + drive)
  FAULT = 3   // transient report, not a command
};

inline const char *poolModeStr(PoolMode m) {
  switch (m) {
  case PoolMode::FILTER:
    return "filter";
  case PoolMode::BACKWASH:
    return "backwash";
  case PoolMode::MOVING:
    return "moving";
  case PoolMode::FAULT:
  default:
    return "fault";
  }
}

struct AppState {
  PoolMode mode = PoolMode::FILTER;
  // Phase 1 fills these; Phase 0 leaves sentinel/default values.
  float psi = 0.0f;
  float conf_sigma = 0.0f;
  float support = 0.0f;
  int rpm = 0;
  int watts = 0;
  const char *valve = "filter";
  const char *pump = "stopped";
  const char *fault = "none";
};

struct ControlState {
  // Backwash run tracking. Timer starts at BACKWASH_RUN entry (spec).
  bool backwashActive = false;
  uint32_t backwashStartMs = 0;
  // Non-blocking 15s pump_stop -> valve_move pre-hold.
  bool preHoldActive = false;
  uint32_t preHoldStartMs = 0;
  PoolMode pendingTarget = PoolMode::FILTER;
};
