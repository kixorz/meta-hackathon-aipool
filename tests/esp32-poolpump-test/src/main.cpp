// ESP32CAM Century pool pump test
// Goal 1: instruct the Century VGreen pump to run at ~1600 rpm.
// Goal 2: evaluate what periodic reminders (if any) it needs to stay there.
//
// Experiment (automatic, starts after a 10 s abort window):
//   PROBE   set_demand_and_run(1600), confirm RUNNING + rpm in band.
//   SOAK    read-only polls (status/rpm/demand/watts) every 10 s x12,
//           then growing SILENT gaps (no bus TX at all: 60/120/300 s)
//           each followed by a single poll. Tests whether the pump needs
//           ANY traffic, or just no demand refresh.
//   REMIND  (only if SOAK sees a dropout) re-issue RUN every 30 s x6 and
//           check the pump holds. Tells us the reminder period that works.
// After the verdict the firmware drops to manual USB control.
//
// Reuses the shared driver in ../../src/century.c + ../../include/century.h
// (pure protocol + retry logic; transport is implemented below on UART2).
// Both are symlinked into this test's src/ so PlatformIO compiles them as-is.
//
// RS485 transceiver wiring (same strapping-safe pins as the other tests):
//   ESP GPIO14 -> DI (driver input)
//   ESP GPIO13 -> RO (receiver input)
//   ESP GND    -> GND (MUST share ground with bus)
//   A <-> A, B <-> B across ESP transceiver <-> pump
// NOTE: keep TX off GPIO12 (strapping pin MTDI — HIGH at reset bricks
// boot/flash). GPIO14 is strapping-safe.
//
// USB debug console = Serial @115200. Keys (loop never blocks on them):
//   R = set demand 1600 + run   S = stop pump (safety)
//   D = set demand only         P = single status poll
//   A = address auto-detect     X = abort auto experiment -> manual
//   H = help

#include <Arduino.h>
#include "century.h"

// ---------- config ----------
#define RS485_TX_PIN 14
#define RS485_RX_PIN 13
#define RS485_BAUD 9600 // assumed VGreen default; pump silent? try 19200/38400
#define RS485_DE_PIN -1 // set to e.g. 15 if your MAX485 needs DE/RE driven
#define TARGET_RPM 1600
#define RPM_BAND 150          // confirm/refresh tolerance around target
#define DROP_RPM (TARGET_RPM - 300) // below this (twice in a row) = dropout
#define CONFIRM_TIMEOUT_MS 45000
#define SOAK_POLL_MS 10000
#define SOAK_POLL_ROUNDS 12
#define REMIND_MS 30000 // reminder period under test in REMIND phase
#define REMIND_CYCLES 6
#define LED_PIN 33 // ESP32CAM onboard LED (active LOW)

static const uint32_t kSilences[] = {60, 120, 300}; // silent-gap schedule (s)

HardwareSerial RS485(2);
static century_client_t g_client;
static century_transport_t g_tr;
static uint8_t g_addr = CENTURY_DEFAULT_ADDR;
static unsigned long g_t0 = 0;

static unsigned long tSecs() { return (millis() - g_t0) / 1000; }

// ---------- transport: century_client <-> UART2 ----------
static void rs485TxEnable(bool tx) {
#if RS485_DE_PIN >= 0
  digitalWrite(RS485_DE_PIN, tx ? HIGH : LOW);
  if (tx)
    delayMicroseconds(200);
#endif
}

// Driver contract: 0 = all bytes written, <0 = error.
static int rs485Write(void *ctx, const uint8_t *buf, size_t len) {
  (void)ctx;
  rs485TxEnable(true);
  size_t n = RS485.write(buf, len);
  RS485.flush();
  delay(5); // >3.5 char silence @9600 for slow transceivers
  rs485TxEnable(false);
  return (n == len) ? 0 : -1;
}

// Driver contract: >0 = bytes, 0 = timeout, <0 = error. The driver calls
// this repeatedly until the frame CRC passes or its deadline expires.
static int rs485Read(void *ctx, uint8_t *buf, size_t max_len,
                      uint32_t timeout_ms) {
  (void)ctx;
  uint32_t t0 = millis();
  while (!RS485.available()) {
    if (millis() - t0 >= timeout_ms)
      return 0;
    delay(1);
  }
  size_t n = 0;
  uint32_t last = millis();
  while (n < max_len && millis() - last < 20) { // end of frame on 20 ms gap
    while (RS485.available() && n < max_len) {
      buf[n++] = (uint8_t)RS485.read();
      last = millis();
    }
    delay(1);
  }
  return (int)n;
}

static void rs485Delay(uint32_t ms) { delay((unsigned long)ms); }
static uint32_t rs485Millis() { return millis(); }

static void clog(void *ctx, century_log_level_t lvl, const char *file,
                 int line, const char *msg) {
  (void)ctx;
  (void)file;
  (void)line;
  char tag = (lvl == CENTURY_LOG_DEBUG) ? 'D' : (lvl == CENTURY_LOG_INFO) ? 'I'
                                            : (lvl == CENTURY_LOG_WARN)  ? 'W'
                                                                        : 'E';
  Serial.printf("[t+%lus][C %c] %s\n", tSecs(), tag, msg);
}

// ---------- helpers ----------
struct PumpState {
  bool ok = false;
  century_status_t status = CENTURY_STATUS_UNKNOWN;
  uint16_t rpm = 0;
  bool rpmOk = false;
  uint16_t demand = 0;
  bool demandOk = false;
  uint16_t watts = 0;
  bool wattsOk = false;
};

// Read-only poll: status + rpm + demand + watts. Never writes demand/run.
static bool pollPump(PumpState &st) {
  st = PumpState();
  century_err_t e = century_get_status(&g_client, &st.status);
  if (e != CENTURY_OK) {
    Serial.printf("[t+%lus][POLL] status FAILED: %s (nack=%s)\n", tSecs(),
                  century_strerror(e),
                  century_nack_str(century_last_nack(&g_client)));
    return false;
  }
  st.rpmOk = (century_read_rpm(&g_client, &st.rpm) == CENTURY_OK);
  st.demandOk =
      (century_read_demand_rpm(&g_client, &st.demand) == CENTURY_OK);
  st.wattsOk =
      (century_read_power_watts(&g_client, &st.watts) == CENTURY_OK);
  st.ok = true;
  return true;
}

static void printPoll(const char *phase, const PumpState &st) {
  Serial.printf("[t+%lus][%s] status=%s rpm=%s demand=%s watts=%s\n", tSecs(),
                phase, century_status_str(st.status),
                st.rpmOk ? String(st.rpm).c_str() : "?",
                st.demandOk ? String(st.demand).c_str() : "?",
                st.wattsOk ? String(st.watts).c_str() : "?");
  digitalWrite(LED_PIN, LOW);
  delay(30);
  digitalWrite(LED_PIN, HIGH);
}

static bool rpmInBand(const PumpState &st) {
  return st.rpmOk && st.rpm + RPM_BAND >= TARGET_RPM &&
         st.rpm <= TARGET_RPM + RPM_BAND;
}

// True if the pump is NOT usefully running at target (call twice to confirm).
static bool looksDropped(const PumpState &st) {
  if (!st.ok)
    return true; // comms loss counts as dropout (re-polled by caller)
  if (st.status == CENTURY_STATUS_FAULT)
    return true;
  if (st.status != CENTURY_STATUS_RUNNING &&
      st.status != CENTURY_STATUS_ACCELERATING)
    return true;
  if (st.rpmOk && st.rpm < DROP_RPM)
    return true;
  return false;
}

static void reportFault() {
  uint8_t code = 0;
  if (century_read_fault_code(&g_client, CENTURY_SENS_PREVFAULT_PAGE,
                              CENTURY_SENS_PREVFAULT_ADDR,
                              &code) == CENTURY_OK) {
    Serial.printf("[t+%lus][FAULT] prev code 0x%02X (%s)\n", tSecs(), code,
                  century_fault_str(code));
  }
}

// ---------- USB manual control ----------
static String s_line;
static bool s_manual = false;

static void handleUsb() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      s_line.trim();
      if (s_line.length() == 1) {
        char k = s_line.charAt(0);
        if (k == 'R' || k == 'r') {
          Serial.printf("[t+%lus][USB] set_demand_and_run(%u)\n", tSecs(),
                        TARGET_RPM);
          century_err_t e = century_set_demand_and_run(&g_client, TARGET_RPM);
          Serial.printf("  -> %s\n", century_strerror(e));
        } else if (k == 'S' || k == 's') {
          Serial.printf("[t+%lus][USB] stop\n", tSecs());
          century_err_t e = century_stop(&g_client);
          Serial.printf("  -> %s\n", century_strerror(e));
        } else if (k == 'D' || k == 'd') {
          Serial.printf("[t+%lus][USB] set_demand(%u) only\n", tSecs(),
                        TARGET_RPM);
          century_err_t e = century_set_demand(&g_client, CENTURY_MODE_SPEED,
                                               TARGET_RPM);
          Serial.printf("  -> %s\n", century_strerror(e));
        } else if (k == 'P' || k == 'p') {
          PumpState st;
          if (pollPump(st))
            printPoll("USB", st);
        } else if (k == 'A' || k == 'a') {
          uint8_t a = 0;
          Serial.printf("[t+%lus][USB] auto-detect...\n", tSecs());
          if (century_auto_detect(&g_client, &a) == CENTURY_OK) {
            g_addr = a;
            Serial.printf("  -> pump at 0x%02X\n", a);
          } else {
            Serial.println("  -> no pump found");
          }
        } else if (k == 'X' || k == 'x') {
          s_manual = true;
          Serial.printf("[t+%lus][USB] auto experiment aborted -> manual\n",
                        tSecs());
        } else if (k == 'H' || k == 'h' || k == '?') {
          Serial.println("keys: R run@1600, S stop, D demand-only, P poll, "
                         "A detect, X manual, H help");
        }
      } else if (s_line.length() > 0) {
        Serial.println("keys: R run@1600, S stop, D demand-only, P poll, "
                       "A detect, X manual, H help");
      }
      s_line = "";
    } else {
      s_line += c;
      if (s_line.length() > 16)
        s_line = "";
    }
  }
}

// ---------- setup: bus, detect, countdown ----------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

#if RS485_DE_PIN >= 0
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
#endif

  Serial.begin(115200);
  delay(500);
  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  g_t0 = millis();

  Serial.println();
  Serial.println("=== ESP32CAM POOL PUMP TEST (Century VGreen) ===");
  Serial.printf("RS485: TX=GPIO%d RX=GPIO%d baud=%d DE_PIN=%d\n", RS485_TX_PIN,
                RS485_RX_PIN, RS485_BAUD, RS485_DE_PIN);
  Serial.printf("Target: %u rpm. Keys: R S D P A X H.\n", TARGET_RPM);

  g_tr.ctx = NULL;
  g_tr.write = rs485Write;
  g_tr.read = rs485Read;
  g_tr.delay_ms = rs485Delay;
  g_tr.millis = rs485Millis;
#if RS485_DE_PIN >= 0
  g_tr.set_tx_enable = NULL; // handled inside rs485Write
#else
  g_tr.set_tx_enable = NULL; // auto-direction transceiver
#endif
  century_client_init(&g_client, g_addr, &g_tr);
  century_client_set_log(&g_client, clog, NULL);

  // Detect: status at default addr, else full auto-detect sweep.
  PumpState st;
  Serial.printf("[t+%lus] probing pump at 0x%02X...\n", tSecs(), g_addr);
  if (pollPump(st)) {
    printPoll("PROBE", st);
  } else {
    Serial.printf("[t+%lus] no reply at 0x%02X, auto-detect sweep...\n",
                  tSecs(), g_addr);
    uint8_t a = 0;
    if (century_auto_detect(&g_client, &a) == CENTURY_OK) {
      g_addr = a;
      Serial.printf("[t+%lus] pump found at 0x%02X\n", tSecs(), a);
      if (pollPump(st))
        printPoll("PROBE", st);
    } else {
      Serial.printf("[t+%lus] NO PUMP FOUND. Check A/B swap, GND, baud, "
                    "pump power + address. Manual mode.\n",
                    tSecs());
      s_manual = true;
    }
  }
  if (st.ok && st.status == CENTURY_STATUS_FAULT)
    reportFault();

  if (!s_manual) {
    Serial.println("Auto experiment starts in 10 s — press any key for "
                   "manual mode (pump keeps current state).");
    unsigned long t0 = millis();
    while (millis() - t0 < 10000) {
      if (Serial.available()) {
        s_manual = true;
        while (Serial.available())
          (void)Serial.read();
        Serial.println("Manual mode. Keys: R S D P A X H.");
        break;
      }
      delay(100);
    }
  }
  Serial.println("===============================");
}

// ---------- auto experiment ----------
enum Phase { PH_PROBE, PH_SOAK_POLL, PH_SOAK_SILENT, PH_REMIND, PH_DONE };

static Phase s_phase = PH_PROBE;
static unsigned long s_nextMs = 0;
static uint8_t s_round = 0;
static uint8_t s_silIdx = 0;
static uint8_t s_badStreak = 0;
static bool s_probeSent = false;
static unsigned long s_probeMs = 0;
static uint8_t s_remindCount = 0;

static void phaseProbe() {
  if (!s_probeSent) {
    Serial.printf("[t+%lus][PROBE] set_demand_and_run(%u)\n", tSecs(),
                  TARGET_RPM);
    century_err_t e = century_set_demand_and_run(&g_client, TARGET_RPM);
    Serial.printf("  -> %s\n", century_strerror(e));
    if (e != CENTURY_OK) {
      Serial.println("[PROBE] demand+run rejected — manual mode. (S = stop)");
      s_phase = PH_DONE;
      s_manual = true;
      return;
    }
    s_probeSent = true;
    s_probeMs = millis();
    s_nextMs = millis() + 3000;
    return;
  }
  if ((long)(millis() - s_nextMs) < 0)
    return;
  PumpState st;
  if (pollPump(st)) {
    printPoll("PROBE", st);
    if (st.status == CENTURY_STATUS_FAULT) {
      reportFault();
      Serial.println("[PROBE] pump FAULT — manual mode. (S = stop)");
      s_phase = PH_DONE;
      s_manual = true;
      return;
    }
    if ((st.status == CENTURY_STATUS_RUNNING && rpmInBand(st))) {
      Serial.printf("[t+%lus][PROBE] CONFIRMED at ~%u rpm, entering SOAK.\n",
                    tSecs(), TARGET_RPM);
      s_phase = PH_SOAK_POLL;
      s_round = 0;
      s_badStreak = 0;
      s_nextMs = millis();
      return;
    }
  }
  if (millis() - s_probeMs > CONFIRM_TIMEOUT_MS) {
    Serial.println("[PROBE] never reached target rpm — manual mode.");
    s_phase = PH_DONE;
    s_manual = true;
    return;
  }
  s_nextMs = millis() + 5000; // keep watching the ramp-up
}

static void enterRemind(const char *why) {
  Serial.printf("[t+%lus][SOAK] DROPOUT (%s) — entering REMIND: RUN every %lus "
                "x%u.\n",
                tSecs(), why, REMIND_MS / 1000, REMIND_CYCLES);
  century_err_t e = century_set_demand_and_run(&g_client, TARGET_RPM);
  Serial.printf("  re-run -> %s\n", century_strerror(e));
  s_phase = PH_REMIND;
  s_remindCount = 0;
  s_badStreak = 0;
  s_nextMs = millis();
}

static void phaseSoakPoll() {
  if ((long)(millis() - s_nextMs) < 0)
    return;
  PumpState st;
  bool ok = pollPump(st);
  if (ok)
    printPoll("SOAK", st);
  if (!ok || looksDropped(st)) {
    if (++s_badStreak >= 2) {
      enterRemind(!ok ? "comms loss x2" : "rpm/status out of band x2");
      return;
    }
    Serial.printf("[t+%lus][SOAK] suspect poll (%u/2), re-checking...\n",
                  tSecs(), s_badStreak);
    s_nextMs = millis() + SOAK_POLL_MS;
    return;
  }
  s_badStreak = 0;
  if (++s_round >= SOAK_POLL_ROUNDS) {
    s_phase = PH_SOAK_SILENT;
    s_silIdx = 0;
    s_nextMs = millis() + (uint32_t)kSilences[0] * 1000;
    Serial.printf("[t+%lus][SOAK] %u read-only polls stable. Silence 1/%u: "
                  "ZERO bus TX for %lus...\n",
                  tSecs(), SOAK_POLL_ROUNDS,
                  (unsigned)(sizeof(kSilences) / sizeof(kSilences[0])),
                  kSilences[0]);
    return;
  }
  s_nextMs = millis() + SOAK_POLL_MS;
}

static void phaseSoakSilent() {
  if ((long)(millis() - s_nextMs) < 0)
    return;
  PumpState st;
  Serial.printf("[t+%lus][SOAK] silence %lus over, polling...\n", tSecs(),
                kSilences[s_silIdx]);
  bool ok = pollPump(st);
  if (ok)
    printPoll("SOAK", st);
  if (!ok || looksDropped(st)) {
    if (++s_badStreak >= 2) {
      enterRemind(!ok ? "silent-gap comms loss x2"
                      : "silent-gap dropout x2");
      return;
    }
    Serial.printf("[t+%lus][SOAK] suspect post-silence poll, re-checking in "
                  "%lus...\n",
                  tSecs(), SOAK_POLL_MS / 1000);
    s_nextMs = millis() + SOAK_POLL_MS;
    return;
  }
  s_badStreak = 0;
  if (++s_silIdx >= sizeof(kSilences) / sizeof(kSilences[0])) {
    Serial.printf("[t+%lus][VERDICT] pump held ~%u rpm through %u read-only "
                  "polls AND silent gaps up to %lus with zero TX.\n",
                  tSecs(), TARGET_RPM, SOAK_POLL_ROUNDS,
                  kSilences[s_silIdx - 1]);
    Serial.println("[VERDICT] No periodic demand reminders needed within "
                   "tested windows. (Read-only polls are bus traffic too — "
                   "silences prove even those aren't required.) Manual mode.");
    s_phase = PH_DONE;
    s_manual = true;
    return;
  }
  s_nextMs = millis() + (uint32_t)kSilences[s_silIdx] * 1000;
  Serial.printf("[t+%lus][SOAK] stable. Silence %u/%u: ZERO bus TX for %lus "
                "...\n",
                tSecs(), s_silIdx + 1,
                (unsigned)(sizeof(kSilences) / sizeof(kSilences[0])),
                kSilences[s_silIdx]);
}

static void phaseRemind() {
  if ((long)(millis() - s_nextMs) < 0)
    return;
  if (s_remindCount > 0) {
    Serial.printf("[t+%lus][REMIND %u/%u] re-issue RUN\n", tSecs(),
                  s_remindCount + 1, REMIND_CYCLES);
    century_err_t e = century_run(&g_client);
    Serial.printf("  -> %s\n", century_strerror(e));
  }
  PumpState st;
  bool ok = pollPump(st);
  if (ok)
    printPoll("REMIND", st);
  if (!ok || looksDropped(st)) {
    Serial.printf("[t+%lus][REMIND] still dropped under %lus reminders — try "
                  "shorter REMIND_MS or demand+run reminders. Manual mode.\n",
                  tSecs(), REMIND_MS / 1000);
    s_phase = PH_DONE;
    s_manual = true;
    return;
  }
  if (++s_remindCount >= REMIND_CYCLES) {
    Serial.printf("[t+%lus][VERDICT] RUN reminder every %lus held ~%u rpm for "
                  "%u cycles.\n",
                  tSecs(), REMIND_MS / 1000, TARGET_RPM, REMIND_CYCLES);
    Serial.println("[VERDICT] Bisect REMIND_MS (60 s, 120 s...) to find the "
                   "max period. Manual mode.");
    s_phase = PH_DONE;
    s_manual = true;
    return;
  }
  s_nextMs = millis() + REMIND_MS;
}

void loop() {
  handleUsb();
  if (s_manual || s_phase == PH_DONE) {
    delay(50);
    return;
  }
  switch (s_phase) {
  case PH_PROBE:
    phaseProbe();
    break;
  case PH_SOAK_POLL:
    phaseSoakPoll();
    break;
  case PH_SOAK_SILENT:
    phaseSoakSilent();
    break;
  case PH_REMIND:
    phaseRemind();
    break;
  case PH_DONE:
    break;
  }
  delay(50);
}
