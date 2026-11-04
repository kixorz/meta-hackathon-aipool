// ESP32CAM valve actuator test (~60 s, then manual)
// Drives the pool valve's 3 relays on the shared 8-ch Modbus RTU relay
// board, bench-verified pattern: BACKWASH = R1 ON + R3 ON (latched),
// FILTER = R2 ON (latched). Sequence: FILTER -> BACKWASH -> FILTER, each
// latched state followed by a hold with readback polls, then a verdict.
//
// Reuses the shared driver in src/relay_board.c + include/relay_board.h
// (pure Modbus RTU frame builder/parser, no UART dependency). Both are
// symlinked into this test's src/ so PlatformIO compiles them as-is.
// The latched-state map below is bench truth (the app's pulse model in
// firmware/app/src/valve.h does NOT match this hardware — see summary).
//
// Safety: pump is NEVER touched by this test (no century code linked).
// Run only with the pump stopped. Transitions are break-before-make
// (R1/R2/R3 all OFF first); the final state is all relays OFF.
//
// RS485 transceiver wiring (same strapping-safe pins as the other tests):
//   ESP GPIO14 -> DI (driver input)
//   ESP GPIO13 -> RO (receiver output)
//   ESP GND    -> GND (MUST share ground with bus)
//   A <-> A, B <-> B across ESP transceiver <-> relay board
//   Relay board powered per its own spec (typically separate 12V supply).
//   120R termination if long cable / errors.
//
// USB debug console = Serial @115200 on /dev/cu.usbserial-1440. At boot the
// auto sequence starts after a 10 s abort window (any key = manual mode).
// Keys (loop never blocks): F = latch filter, B = latch backwash,
// O = all relays OFF (safe state), S = read + print bitmap,
// R = re-run the 1-min auto sequence, H = help.

#include <Arduino.h>
#include "relay_board.h"

// ---------- valve map (bench truth: latched states) ----------
// BACKWASH: R1 ON + R3 ON (R2 OFF). FILTER: R2 ON (R1,R3 OFF).
// Bitmaps: FILTER = 0x02, BACKWASH = 0x05, all OFF = 0x00.
#define VALVE_RELAY_A 1
#define VALVE_RELAY_B 2
#define VALVE_RELAY_C 3
#define VALVE_BITMAP_FILTER (RELAY_BOARD_BIT(2))
#define VALVE_BITMAP_BACKWASH (RELAY_BOARD_BIT(1) | RELAY_BOARD_BIT(3))

// ---------- config ----------
#define RS485_TX_PIN 14
#define RS485_RX_PIN 13
#define RS485_BAUD RELAY_BOARD_BAUD // 9600 8N1, must match relay board
#define RS485_DE_PIN -1             // set to e.g. 15 if your MAX485 needs DE/RE driven
#define RELAY_SLAVE RELAY_BOARD_SLAVE_ADDR
#define HOLD_MS 20000      // hold per phase incl. ~20 s valve travel window
#define HOLD_POLL_MS 2000  // readback cadence during holds
#define MODBUS_TIMEOUT_MS 300
#define LED_PIN 33 // ESP32CAM onboard LED (active LOW)

HardwareSerial RS485(2);
static unsigned long g_t0 = 0;
static bool s_manual = false;
static String s_line;

static void handleUsb(); // loop keys; defined below, used during holds

static unsigned long tSecs() { return (millis() - g_t0) / 1000; }

static void rs485TxEnable(bool tx) {
#if RS485_DE_PIN >= 0
  digitalWrite(RS485_DE_PIN, tx ? HIGH : LOW);
  if (tx)
    delayMicroseconds(200);
#endif
}

// Half-duplex Modbus RTU transaction: send tx, collect the reply.
// Returns reply length; frame ends on ~20 ms inter-byte silence or timeout.
static size_t modbusTxn(const uint8_t *tx, size_t txLen, uint8_t *rx,
                        size_t rxCap) {
  while (RS485.available())
    (void)RS485.read(); // resync: drop stale bus bytes before asking

  rs485TxEnable(true);
  RS485.write(tx, txLen);
  RS485.flush(); // wait for shift register to empty
  delay(5);      // >3.5 char silence @9600 (~4 ms) for slow transceivers
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

static void printHex(const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++)
    Serial.printf("%02X ", buf[i]);
}

// Function 0x05: turn one relay ON/OFF. True if the board echoes the request.
static bool setRelay(uint8_t relay, bool on) {
  uint8_t tx[RELAY_BOARD_WRITE_REQ_LEN];
  uint8_t rx[16];
  size_t txLen = relay_board_build_write_coil(RELAY_SLAVE, relay, on, tx);
  if (txLen == 0)
    return false;
  size_t n = modbusTxn(tx, txLen, rx, sizeof(rx));
  bool ok = relay_board_is_write_echo(tx, rx, n);
  Serial.printf("  R%u %s: tx ", relay, on ? "ON " : "OFF");
  printHex(tx, txLen);
  Serial.printf(" rx(%u): ", (unsigned)n);
  printHex(rx, n);
  Serial.printf(" %s\n", ok ? "OK" : "FAIL");
  if (ok) {
    digitalWrite(LED_PIN, LOW);
    delay(30);
    digitalWrite(LED_PIN, HIGH);
  }
  return ok;
}

// Function 0x01: read all 8 coils into *bitmap. True on valid reply.
static bool readRelays(uint8_t &bitmap) {
  uint8_t tx[RELAY_BOARD_READ_REQ_LEN];
  uint8_t rx[16];
  size_t txLen = relay_board_build_read_coils(RELAY_SLAVE, tx);
  if (txLen == 0)
    return false;
  size_t n = modbusTxn(tx, txLen, rx, sizeof(rx));
  if (!relay_board_parse_coils(rx, n, RELAY_SLAVE, &bitmap)) {
    Serial.printf("  read coils FAILED rx(%u): ", (unsigned)n);
    printHex(rx, n);
    Serial.println();
    return false;
  }
  return true;
}

// Break-before-make: valve relays R1/R2/R3 all OFF. Other relays untouched.
static bool valveRelaysOff() {
  bool ok = setRelay(VALVE_RELAY_A, false);
  ok &= setRelay(VALVE_RELAY_B, false);
  ok &= setRelay(VALVE_RELAY_C, false);
  return ok;
}

// State check: readback bitmap must equal expected exactly. Relays 4-8 are
// left alone by this test (boot safety sweep turns them OFF first).
static bool checkState(uint8_t expect, const char *phase) {
  uint8_t bm = 0xFF;
  bool ok = readRelays(bm) && bm == expect;
  Serial.printf("[t+%lus][%s] bitmap=0x%02X expect=0x%02X %s\n", tSecs(),
                phase, bm, expect, ok ? "MATCH" : "*** MISMATCH ***");
  return ok;
}

// Latch FILTER: break-before-make, then R2 ON. Expect bitmap 0x02.
static bool gotoFilter(const char *phase) {
  Serial.printf("[t+%lus][%s] all valve relays OFF (break-before-make)...\n",
                tSecs(), phase);
  bool ok = valveRelaysOff();
  Serial.printf("[t+%lus][%s] latch FILTER: R2 ON...\n", tSecs(), phase);
  ok &= setRelay(VALVE_RELAY_B, true);
  Serial.printf("[t+%lus][%s] latch %s\n", tSecs(), phase,
                ok ? "echoed" : "*** WRITE FAILED ***");
  return ok;
}

// Latch BACKWASH: break-before-make, then R1 ON + R3 ON. Expect 0x05.
static bool gotoBackwash(const char *phase) {
  Serial.printf("[t+%lus][%s] all valve relays OFF (break-before-make)...\n",
                tSecs(), phase);
  bool ok = valveRelaysOff();
  Serial.printf("[t+%lus][%s] latch BACKWASH: R1 ON + R3 ON...\n", tSecs(),
                phase);
  ok &= setRelay(VALVE_RELAY_A, true);
  ok &= setRelay(VALVE_RELAY_C, true);
  Serial.printf("[t+%lus][%s] latch %s\n", tSecs(), phase,
                ok ? "echoed" : "*** WRITE FAILED ***");
  return ok;
}

// Latched-hold: poll readback every HOLD_POLL_MS for HOLD_MS. True if the
// latched bitmap held the whole time.
static bool holdState(uint8_t expect, const char *phase) {
  Serial.printf("[t+%lus][%s] holding 0x%02X for %u s (readback every %u ms)...\n",
                tSecs(), phase, expect, HOLD_MS / 1000, HOLD_POLL_MS);
  unsigned long tEnd = millis() + HOLD_MS;
  bool ok = checkState(expect, phase);
  while ((long)(millis() - tEnd) < 0) {
    handleUsb();
    delay(HOLD_POLL_MS);
    ok &= checkState(expect, phase);
  }
  Serial.printf("[t+%lus][%s] hold %s\n", tSecs(), phase,
                ok ? "HELD" : "*** DROPPED/READ FAILED ***");
  return ok;
}

// FILTER phase: latch R2, hold 0x02.
static bool phaseFilter(const char *tag) {
  Serial.printf("[t+%lus]=== %s: -> FILTER (R2 on) ===\n", tSecs(), tag);
  bool ok = gotoFilter(tag);
  ok &= holdState(VALVE_BITMAP_FILTER, tag);
  Serial.printf("[t+%lus]=== %s: FILTER %s ===\n", tSecs(), tag,
                ok ? "PASS" : "FAIL");
  return ok;
}

// BACKWASH phase: latch R1+R3, hold 0x05.
static bool phaseBackwash(const char *tag) {
  Serial.printf("[t+%lus]=== %s: -> BACKWASH (R1+R3 on) ===\n", tSecs(), tag);
  bool ok = gotoBackwash(tag);
  ok &= holdState(VALVE_BITMAP_BACKWASH, tag);
  Serial.printf("[t+%lus]=== %s: BACKWASH %s ===\n", tSecs(), tag,
                ok ? "PASS" : "FAIL");
  return ok;
}

static void runAuto() {
  bool f1 = phaseFilter("AUTO-1");
  bool b = phaseBackwash("AUTO-2");
  bool f2 = phaseFilter("AUTO-3");
  Serial.printf("[t+%lus][SAFE] end state: all valve relays OFF\n", tSecs());
  valveRelaysOff();
  Serial.println("===============================");
  Serial.printf("[t+%lus][VERDICT] FILTER-1:%s BACKWASH:%s FILTER-2:%s => %s\n",
                tSecs(), f1 ? "PASS" : "FAIL", b ? "PASS" : "FAIL",
                f2 ? "PASS" : "FAIL",
                (f1 && b && f2) ? "ALL PASS" : "*** FAILURES PRESENT ***");
  Serial.println("Manual mode. Keys: F B O S R H. (Pump is never touched.)");
}

static void allRelaysOff() {
  Serial.printf("[t+%lus][SAFE] all relays OFF\n", tSecs());
  for (uint8_t r = 1; r <= RELAY_BOARD_NUM_RELAYS; r++)
    setRelay(r, false);
}

static void handleUsb() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      s_line.trim();
      if (s_line.length() == 1) {
        char k = s_line.charAt(0);
        if (k == 'F' || k == 'f') {
          phaseFilter("MANUAL");
        } else if (k == 'B' || k == 'b') {
          phaseBackwash("MANUAL");
        } else if (k == 'O' || k == 'o' || k == '0') {
          allRelaysOff();
        } else if (k == 'S' || k == 's') {
          uint8_t bm = 0;
          if (readRelays(bm))
            Serial.printf("[t+%lus][STATE] bitmap=0x%02X\n", tSecs(), bm);
        } else if (k == 'R' || k == 'r') {
          runAuto();
        } else if (k == 'H' || k == 'h' || k == '?') {
          Serial.println("keys: F latch filter (R2), B latch backwash "
                         "(R1+R3), O all off, S state, R re-run auto, H help");
        } else {
          Serial.println("keys: F latch filter (R2), B latch backwash "
                         "(R1+R3), O all off, S state, R re-run auto, H help");
        }
      } else if (s_line.length() > 0) {
        Serial.println("keys: F latch filter (R2), B latch backwash "
                       "(R1+R3), O all off, S state, R re-run auto, H help");
      }
      s_line = "";
    } else {
      s_line += c;
      if (s_line.length() > 16)
        s_line = "";
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED off (active low)

#if RS485_DE_PIN >= 0
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW); // start in receive mode
#endif

  Serial.begin(115200);
  delay(500);
  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  g_t0 = millis();

  Serial.println();
  Serial.println("=== ESP32CAM VALVE TEST (relay board, ~60 s) ===");
  Serial.printf("RS485: TX=GPIO%d RX=GPIO%d baud=%d DE_PIN=%d\n", RS485_TX_PIN,
                RS485_RX_PIN, RS485_BAUD, RS485_DE_PIN);
  Serial.println("Map: FILTER=R2 on, BACKWASH=R1+R3 on (latched states).");
  Serial.println("Safety: all relays OFF first. Pump is NEVER touched here.");
  allRelaysOff();

  if (!s_manual) {
    Serial.println("Auto FILTER->BACKWASH->FILTER starts in 10 s — press any "
                   "key for manual mode.");
    unsigned long t0 = millis();
    while (millis() - t0 < 10000) {
      if (Serial.available()) {
        s_manual = true;
        while (Serial.available())
          (void)Serial.read();
        Serial.println("Manual mode. Keys: F B O S R H.");
        break;
      }
      delay(100);
    }
  }
  if (!s_manual) {
    runAuto();
    s_manual = true; // auto runs once; USB owns the loop afterwards
  }
  Serial.println("===============================");
}

void loop() {
  handleUsb(); // cheap USB commands between phases; loop never blocks on them
  delay(50);
}
