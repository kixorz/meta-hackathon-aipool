// ESP32CAM combined test: relay board + pool pump on ONE RS485 bus.
// The 8-ch Modbus relay board (slave 1) and the Century VGreen pump
// (addr 0x15) share UART2 @9600 8N1. Each ignores the other's frames by
// address/CRC, so the ESP32CAM can master both sequentially.
//
// Loop: sweep relays R1..R8 (ON -> verify -> OFF, like esp32-relays-test)
// while holding the pump at ~1600 rpm via set_demand_and_run() refreshes
// every PUMP_REFRESH_MS (poolpump test showed the volatile demand register
// decays without refresh, so we re-arm it; rpm held regardless).
//
// Reuses the shared drivers (pure protocol, no UART dependency):
//   ../../src/relay_board.c + ../../include/relay_board.h  (Modbus 0x01/0x05)
//   ../../src/century.c + ../../include/century.h (+ _config.h) (VGreen)
// All symlinked into this test's src/ so PlatformIO compiles them as-is.
// Fix protocol bugs in src/ + include/, not here.
//
// RS485 transceiver wiring (same strapping-safe pins as the other tests):
//   ESP GPIO14 -> DI (driver input)
//   ESP GPIO13 -> RO (receiver input)
//   ESP GND    -> GND (MUST share ground with bus)
//   A <-> A, B <-> B across ESP transceiver <-> relay board <-> pump
//   Relay board powered per its own spec (separate supply); only A/B/GND shared.
// NOTE: keep TX off GPIO12 (strapping pin MTDI — HIGH at reset bricks
// boot/flash). GPIO14 is strapping-safe.
//
// USB debug console = Serial @115200. Keys (loop never blocks):
//   relays: 1..8 click once, A all on, O (or 0) all off, G toggle sweep
//   pump:   R demand+run@1600, S stop pump, D demand-only, P poll both
//   H help

#include <Arduino.h>
#include "relay_board.h"
#include "century.h"

// ---------- config ----------
#define RS485_TX_PIN 14
#define RS485_RX_PIN 13
#define RS485_BAUD 9600 // both slaves run 9600 8N1 on this bus
#define RS485_DE_PIN -1 // set to e.g. 15 if your MAX485 needs DE/RE driven
#define TARGET_RPM 1600
#define RPM_BAND 150
#define DROP_RPM (TARGET_RPM - 300)
#define CLICK_ON_MS 600
#define CLICK_OFF_MS 300
#define NEXT_RELAY_MS 200
#define MODBUS_TIMEOUT_MS 300
#define PUMP_REFRESH_MS 60000 // re-issue demand+run this often (re-arms demand reg)
#define CONFIRM_TIMEOUT_MS 45000
#define LED_PIN 33 // ESP32CAM onboard LED (active LOW)

HardwareSerial RS485(2);
static century_client_t g_client;
static century_transport_t g_tr;
static uint8_t g_pumpAddr = CENTURY_DEFAULT_ADDR;
static bool g_pumpPresent = false;
static bool g_sweep = true;
static uint8_t g_relay = 1;
static uint8_t g_badStreak = 0;
static unsigned long g_t0 = 0;
static unsigned long g_lastRefresh = 0;

static unsigned long tSecs() { return (millis() - g_t0) / 1000; }

// ---------- shared bus helpers ----------
static void rs485TxEnable(bool tx) {
#if RS485_DE_PIN >= 0
  digitalWrite(RS485_DE_PIN, tx ? HIGH : LOW);
  if (tx)
    delayMicroseconds(200);
#endif
}

static void printHex(const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++)
    Serial.printf("%02X ", buf[i]);
}

// ---------- relay board (Modbus, slave 1) ----------
static size_t modbusTxn(const uint8_t *tx, size_t txLen, uint8_t *rx,
                        size_t rxCap) {
  while (RS485.available())
    (void)RS485.read(); // resync before asking

  rs485TxEnable(true);
  RS485.write(tx, txLen);
  RS485.flush();
  delay(5); // >3.5 char silence @9600 for slow transceivers
  rs485TxEnable(false);

  size_t n = 0;
  unsigned long t0 = millis(), lastByte = t0;
  while (millis() - t0 < MODBUS_TIMEOUT_MS) {
    while (RS485.available()) {
      int c = RS485.read();
      if (c < 0)
        break;
      if (n < rxCap)
        rx[n++] = (uint8_t)c;
      lastByte = millis();
    }
    if (n > 0 && millis() - lastByte > 20)
      break; // end of frame
    delay(1);
  }
  return n;
}

static bool setRelay(uint8_t relay, bool on) {
  uint8_t tx[RELAY_BOARD_WRITE_REQ_LEN];
  uint8_t rx[16];
  size_t txLen = relay_board_build_write_coil(RELAY_BOARD_SLAVE_ADDR, relay,
                                              on, tx);
  if (txLen == 0)
    return false;
  size_t n = modbusTxn(tx, txLen, rx, sizeof(rx));
  Serial.printf("  tx: ");
  printHex(tx, txLen);
  Serial.printf(" rx(%u): ", (unsigned)n);
  printHex(rx, n);
  Serial.println();
  return relay_board_is_write_echo(tx, rx, n);
}

static bool readRelays(uint8_t *bitmap) {
  uint8_t tx[RELAY_BOARD_READ_REQ_LEN];
  uint8_t rx[16];
  size_t txLen = relay_board_build_read_coils(RELAY_BOARD_SLAVE_ADDR, tx);
  if (txLen == 0)
    return false;
  size_t n = modbusTxn(tx, txLen, rx, sizeof(rx));
  if (!relay_board_parse_coils(rx, n, RELAY_BOARD_SLAVE_ADDR, bitmap)) {
    Serial.printf("  relay read FAILED rx(%u): ", (unsigned)n);
    printHex(rx, n);
    Serial.println();
    return false;
  }
  return true;
}

static void blink(int ms) {
  digitalWrite(LED_PIN, LOW);
  delay(ms);
  digitalWrite(LED_PIN, HIGH);
}

static bool clickRelay(uint8_t relay) {
  uint8_t bm = 0;
  bool ok = true;

  Serial.printf("[t+%lus][R%u ON ] ", tSecs(), relay);
  bool wOn = setRelay(relay, true);
  bool rOn = readRelays(&bm);
  bool expectOn = rOn && relay_board_get(bm, relay);
  Serial.printf(" -> write:%s readback:0x%02X(%s) %s\n", wOn ? "OK" : "FAIL",
                bm, rOn ? "ok" : "BAD",
                (wOn && expectOn) ? "CLICK" : "*** CHECK BOARD ***");
  ok &= (wOn && expectOn);
  blink(50);
  delay(CLICK_ON_MS);

  Serial.printf("[t+%lus][R%u OFF] ", tSecs(), relay);
  bool wOff = setRelay(relay, false);
  bool rOff = readRelays(&bm);
  bool expectOff = rOff && !relay_board_get(bm, relay);
  Serial.printf(" -> write:%s readback:0x%02X(%s) %s\n", wOff ? "OK" : "FAIL",
                bm, rOff ? "ok" : "BAD",
                (wOff && expectOff) ? "CLACK" : "*** CHECK BOARD ***");
  ok &= (wOff && expectOff);
  blink(50);
  delay(CLICK_OFF_MS);
  return ok;
}

static void allRelays(bool on) {
  Serial.printf("[t+%lus][RELAY ALL %s]\n", tSecs(), on ? "ON" : "OFF");
  for (uint8_t r = 1; r <= RELAY_BOARD_NUM_RELAYS; r++) {
    bool ok = setRelay(r, on);
    Serial.printf("  R%u %s:%s\n", r, on ? "ON " : "OFF", ok ? "OK" : "FAIL");
  }
}

// ---------- pump (Century, addr g_pumpAddr) ----------
static int pumpWrite(void *ctx, const uint8_t *buf, size_t len) {
  (void)ctx;
  rs485TxEnable(true);
  size_t n = RS485.write(buf, len);
  RS485.flush();
  delay(5);
  rs485TxEnable(false);
  return (n == len) ? 0 : -1;
}

static int pumpRead(void *ctx, uint8_t *buf, size_t max_len,
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
  while (n < max_len && millis() - last < 20) {
    while (RS485.available() && n < max_len) {
      buf[n++] = (uint8_t)RS485.read();
      last = millis();
    }
    delay(1);
  }
  return (int)n;
}

static void pumpDelay(uint32_t ms) { delay((unsigned long)ms); }
static uint32_t pumpMillis() { return millis(); }

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

static bool pollPump(PumpState &st) {
  st = PumpState();
  century_err_t e = century_get_status(&g_client, &st.status);
  if (e != CENTURY_OK) {
    Serial.printf("[t+%lus][PUMP] status FAILED: %s (nack=%s)\n", tSecs(),
                  century_strerror(e),
                  century_nack_str(century_last_nack(&g_client)));
    return false;
  }
  st.rpmOk = (century_read_rpm(&g_client, &st.rpm) == CENTURY_OK);
  st.demandOk = (century_read_demand_rpm(&g_client, &st.demand) == CENTURY_OK);
  st.wattsOk = (century_read_power_watts(&g_client, &st.watts) == CENTURY_OK);
  st.ok = true;
  return true;
}

static void printPump(const char *phase, const PumpState &st) {
  Serial.printf("[t+%lus][%s] pump status=%s rpm=%s demand=%s watts=%s\n",
                tSecs(), phase, century_status_str(st.status),
                st.rpmOk ? String(st.rpm).c_str() : "?",
                st.demandOk ? String(st.demand).c_str() : "?",
                st.wattsOk ? String(st.watts).c_str() : "?");
}

static bool pumpDropped(const PumpState &st) {
  if (!st.ok)
    return true;
  if (st.status == CENTURY_STATUS_FAULT)
    return true;
  if (st.status != CENTURY_STATUS_RUNNING &&
      st.status != CENTURY_STATUS_ACCELERATING)
    return true;
  if (st.rpmOk && st.rpm < DROP_RPM)
    return true;
  return false;
}

// Re-arm demand + run, then verify. Returns true if pump usefully running.
static bool pumpRefresh() {
  Serial.printf("[t+%lus][PUMP] refresh set_demand_and_run(%u)\n", tSecs(),
                TARGET_RPM);
  century_err_t e = century_set_demand_and_run(&g_client, TARGET_RPM);
  Serial.printf("  -> %s\n", century_strerror(e));
  PumpState st;
  bool ok = pollPump(st);
  if (ok)
    printPump("PUMP", st);
  if (st.ok && st.status == CENTURY_STATUS_FAULT) {
    uint8_t code = 0;
    if (century_read_fault_code(&g_client, CENTURY_SENS_PREVFAULT_PAGE,
                                CENTURY_SENS_PREVFAULT_ADDR,
                                &code) == CENTURY_OK)
      Serial.printf("[t+%lus][PUMP] FAULT prev code 0x%02X (%s)\n", tSecs(),
                    code, century_fault_str(code));
  }
  return ok && !pumpDropped(st);
}

// ---------- USB ----------
static String s_line;

static void handleUsb() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      s_line.trim();
      if (s_line.length() == 1) {
        char k = s_line.charAt(0);
        if (k >= '1' && k <= '8') {
          clickRelay((uint8_t)(k - '0'));
        } else if (k == 'A' || k == 'a') {
          allRelays(true);
        } else if (k == 'O' || k == 'o' || k == '0') {
          allRelays(false);
        } else if (k == 'G' || k == 'g') {
          g_sweep = !g_sweep;
          Serial.printf("[t+%lus][USB] relay sweep %s\n", tSecs(),
                        g_sweep ? "ON" : "OFF");
        } else if (k == 'R' || k == 'r') {
          if (g_pumpPresent)
            pumpRefresh();
          else
            Serial.println("[USB] no pump on bus");
        } else if (k == 'S' || k == 's') {
          if (g_pumpPresent) {
            century_err_t e = century_stop(&g_client);
            Serial.printf("[t+%lus][USB] pump stop -> %s\n", tSecs(),
                          century_strerror(e));
          } else {
            Serial.println("[USB] no pump on bus");
          }
        } else if (k == 'D' || k == 'd') {
          if (g_pumpPresent) {
            century_err_t e = century_set_demand(&g_client, CENTURY_MODE_SPEED,
                                                 TARGET_RPM);
            Serial.printf("[t+%lus][USB] pump demand-only -> %s\n", tSecs(),
                          century_strerror(e));
          } else {
            Serial.println("[USB] no pump on bus");
          }
        } else if (k == 'P' || k == 'p') {
          uint8_t bm = 0;
          if (readRelays(&bm))
            Serial.printf("[t+%lus][USB] relays bitmap=0x%02X\n", tSecs(), bm);
          if (g_pumpPresent) {
            PumpState st;
            if (pollPump(st))
              printPump("USB", st);
          }
        } else {
          Serial.println("keys: 1-8 click, A all on, O all off, G sweep, "
                         "R pump run, S pump stop, D demand, P poll both");
        }
      } else if (s_line.length() > 0) {
        Serial.println("keys: 1-8 click, A all on, O all off, G sweep, "
                       "R pump run, S pump stop, D demand, P poll both");
      }
      s_line = "";
    } else {
      s_line += c;
      if (s_line.length() > 16)
        s_line = "";
    }
  }
}

// ---------- setup / loop ----------
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
  Serial.println("=== ESP32CAM COMBINED TEST (relays + pump, one bus) ===");
  Serial.printf("RS485: TX=GPIO%d RX=GPIO%d baud=%d DE_PIN=%d\n", RS485_TX_PIN,
                RS485_RX_PIN, RS485_BAUD, RS485_DE_PIN);

  g_tr.ctx = NULL;
  g_tr.write = pumpWrite;
  g_tr.read = pumpRead;
  g_tr.delay_ms = pumpDelay;
  g_tr.millis = pumpMillis;
  g_tr.set_tx_enable = NULL; // auto-direction (or handled in pumpWrite)
  century_client_init(&g_client, g_pumpAddr, &g_tr);
  century_client_set_log(&g_client, clog, NULL);

  Serial.println("Safety: switching ALL relays OFF first.");
  allRelays(false);

  PumpState st;
  Serial.printf("[t+%lus] probing pump at 0x%02X...\n", tSecs(), g_pumpAddr);
  if (pollPump(st)) {
    printPump("PROBE", st);
    g_pumpPresent = true;
  } else {
    Serial.printf("[t+%lus] no reply at 0x%02X, auto-detect sweep...\n",
                  tSecs(), g_pumpAddr);
    uint8_t a = 0;
    if (century_auto_detect(&g_client, &a) == CENTURY_OK) {
      g_pumpAddr = a;
      g_pumpPresent = true;
      Serial.printf("[t+%lus] pump found at 0x%02X\n", tSecs(), a);
    } else {
      Serial.println("[t+?] NO PUMP — continuing relays-only. (R/S/D/P "
                     "will report no pump.)");
    }
  }

  if (g_pumpPresent) {
    Serial.printf("[t+%lus] starting pump at %u rpm...\n", tSecs(), TARGET_RPM);
    century_err_t e = century_set_demand_and_run(&g_client, TARGET_RPM);
    Serial.printf("  -> %s\n", century_strerror(e));
    unsigned long tStart = millis();
    bool confirmed = false;
    while (millis() - tStart < CONFIRM_TIMEOUT_MS) {
      handleUsb();
      delay(3000);
      if (pollPump(st)) {
        printPump("PROBE", st);
        if (st.status == CENTURY_STATUS_RUNNING && st.rpmOk &&
            st.rpm + RPM_BAND >= TARGET_RPM && st.rpm <= TARGET_RPM + RPM_BAND) {
          confirmed = true;
          break;
        }
        if (st.status == CENTURY_STATUS_FAULT) {
          Serial.println("[PROBE] pump FAULT — continuing relays-only.");
          g_pumpPresent = false;
          break;
        }
      }
    }
    if (g_pumpPresent && !confirmed)
      Serial.println("[PROBE] pump never reached target — keeping refresh "
                     "loop anyway; watch the log.");
    g_lastRefresh = millis();
  }

  Serial.println("Loop: relay sweep R1..R8 + pump refresh every 60 s. Keys: "
                 "1-8 A O G R S D P.");
  Serial.println("===============================");
}

void loop() {
  handleUsb();

  if (g_sweep) {
    bool ok = clickRelay(g_relay);
    if (!ok)
      Serial.printf("[t+%lus][R%u] *** relay no-reply — check board ***\n",
                    tSecs(), g_relay);
    g_relay = (uint8_t)(g_relay % RELAY_BOARD_NUM_RELAYS + 1);
    delay(NEXT_RELAY_MS);
  } else {
    delay(50);
  }

  if (g_pumpPresent && millis() - g_lastRefresh >= PUMP_REFRESH_MS) {
    g_lastRefresh = millis();
    if (pumpRefresh()) {
      g_badStreak = 0;
    } else if (++g_badStreak >= 2) {
      Serial.printf("[t+%lus][PUMP] *** dropout x2 under refresh — check "
                    "pump/bus ***\n",
                    tSecs());
      g_badStreak = 0; // keep trying; don't spam
    }
  }
}
