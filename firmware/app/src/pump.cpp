// Pump driver: century client on shared bus. All UART access takes
// g_busMutex via rs485-style byte transport (never bypass the mutex).

#include "pump.h"

#include <Arduino.h>

#include "bus.h"
#include "century.h"

#define PUMP_REFRESH_MS 60000
#define PUMP_ADDR CENTURY_DEFAULT_ADDR // 0x15

static century_client_t s_client;
static century_transport_t s_tr;
static bool s_present = false;
static unsigned long s_lastRefreshMs = 0;
static uint8_t s_badStreak = 0;
static char s_err[64] = {0};

static int pumpWrite(void *ctx, const uint8_t *buf, size_t len) {
  (void)ctx;
  if (!busTake(1500))
    return -1;
  while (RS485Bus.available())
    (void)RS485Bus.read();
#if AIPOOL_RS485_DE_PIN >= 0
  digitalWrite(AIPOOL_RS485_DE_PIN, HIGH);
  delayMicroseconds(200);
#endif
  size_t n = RS485Bus.write(buf, len);
  RS485Bus.flush();
  delay(5);
#if AIPOOL_RS485_DE_PIN >= 0
  digitalWrite(AIPOOL_RS485_DE_PIN, LOW);
#endif
  busGive();
  return (n == len) ? 0 : -1;
}

static int pumpRead(void *ctx, uint8_t *buf, size_t maxLen,
                    uint32_t timeoutMs) {
  (void)ctx;
  // NB: mutex already released by pumpWrite; take it for the read window.
  if (!busTake(timeoutMs + 500))
    return -1;
  uint32_t t0 = millis();
  while (!RS485Bus.available()) {
    if (millis() - t0 >= timeoutMs) {
      busGive();
      return 0;
    }
    delay(1);
  }
  size_t n = 0;
  uint32_t last = millis();
  while (n < maxLen && millis() - last < 20) {
    while (RS485Bus.available() && n < maxLen) {
      buf[n++] = (uint8_t)RS485Bus.read();
      last = millis();
    }
    delay(1);
  }
  busGive();
  return (int)n;
}

static void pumpDelayMs(uint32_t ms) { delay(ms); }
static uint32_t pumpMillis() { return millis(); }

static char s_lastClog[96] = {0};
static unsigned long s_lastClogMs = 0;

static void clog(void *ctx, century_log_level_t lvl, const char *file,
                 int line, const char *msg) {
  (void)ctx;
  (void)file;
  (void)line;
  if (lvl < CENTURY_LOG_WARN || !msg)
    return;
  // Rate-limit duplicate driver warnings (absent pump = same timeout every
  // poll): first occurrence + one reminder per minute, always on change.
  unsigned long now = millis();
  if (strcmp(msg, s_lastClog) != 0 || now - s_lastClogMs >= 60000) {
    strncpy(s_lastClog, msg, sizeof(s_lastClog) - 1);
    s_lastClog[sizeof(s_lastClog) - 1] = '\0';
    s_lastClogMs = now;
    Serial.printf("[pump %c] %s\n", lvl == CENTURY_LOG_WARN ? 'W' : 'E', msg);
  }
}

void pumpInit(uint16_t targetRpm) {
  (void)targetRpm;
  s_tr.ctx = nullptr;
  s_tr.write = pumpWrite;
  s_tr.read = pumpRead;
  s_tr.delay_ms = pumpDelayMs;
  s_tr.millis = pumpMillis;
  s_tr.set_tx_enable = nullptr; // handled inline (auto-direction)
  century_client_init(&s_client, PUMP_ADDR, &s_tr);
  century_client_set_log(&s_client, clog, nullptr);
  s_present = false;
  s_lastRefreshMs = 0;
  s_badStreak = 0;
  s_err[0] = '\0';
}

const char *pumpLastError() { return s_err; }

bool pumpPoll(PumpReading &out) {
  out = PumpReading();
  out.present = s_present;
  century_status_t st = CENTURY_STATUS_UNKNOWN;
  century_err_t e = century_get_status(&s_client, &st);
  if (e != CENTURY_OK) {
    snprintf(s_err, sizeof(s_err), "no reply: status %s (A/B? GND? power? 0x15)",
             century_strerror(e));
    out.err = s_err;
    return false;
  }
  out.status = st;
  out.rpmOk = (century_read_rpm(&s_client, &out.rpm) == CENTURY_OK);
  out.demandOk =
      (century_read_demand_rpm(&s_client, &out.demand) == CENTURY_OK);
  out.wattsOk =
      (century_read_power_watts(&s_client, &out.watts) == CENTURY_OK);
  if (st == CENTURY_STATUS_FAULT) {
    uint8_t code = 0;
    if (century_read_fault_code(&s_client, CENTURY_SENS_PREVFAULT_PAGE,
                                CENTURY_SENS_PREVFAULT_ADDR,
                                &code) == CENTURY_OK) {
      out.faultCode = code;
    }
  }
  out.ok = true;
  out.present = true;
  s_present = true;
  out.err = "none";
  return true;
}

bool pumpStop() {
  century_err_t e = century_stop(&s_client);
  if (e != CENTURY_OK) {
    snprintf(s_err, sizeof(s_err), "stop %s", century_strerror(e));
    return false;
  }
  s_lastRefreshMs = 0; // force re-arm on next run
  return true;
}

bool pumpRun(uint16_t rpm) {
  century_err_t e = century_set_demand_and_run(&s_client, rpm);
  if (e != CENTURY_OK) {
    snprintf(s_err, sizeof(s_err), "run %s", century_strerror(e));
    return false;
  }
  s_lastRefreshMs = millis();
  s_badStreak = 0;
  return true;
}

bool pumpRefreshIfDue(uint16_t targetRpm) {
  if (!s_present)
    return false;
  if (millis() - s_lastRefreshMs < PUMP_REFRESH_MS)
    return true;
  s_lastRefreshMs = millis();
  if (!pumpRun(targetRpm) && ++s_badStreak >= 2) {
    s_badStreak = 0; // keep trying; don't spam (combined-test pattern)
    return false;
  }
  return true;
}

bool pumpStoppedOk() {
  PumpReading r;
  if (!pumpPoll(r))
    return false; // no reply: NOT safe to move valve
  return r.status == CENTURY_STATUS_STOPPED;
}
