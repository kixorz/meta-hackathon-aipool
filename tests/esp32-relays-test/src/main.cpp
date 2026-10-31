// ESP32CAM relay board test
// Drives the 8-channel Modbus RTU relay board over RS485, iterating
// relays 1..8 forever: ON (click) -> verify -> OFF (click) -> next.
//
// Reuses the shared driver in ../../src/relay_board.{c,h} (pure-C Modbus
// RTU frame builder/parser, no UART dependency). The two files are
// symlinked into this test's src/ so PlatformIO compiles them as-is;
// all protocol knowledge (function codes, CRC, coil map) lives there.
//
// RS485 transceiver wiring (same strapping-safe pins as esp32-rs485-test):
//   ESP GPIO14 -> DI (driver input)
//   ESP GPIO13 -> RO (receiver output)
//   ESP GND    -> GND (MUST share ground with bus, even though RS485 is differential)
//   A <-> A, B <-> B across ESP transceiver <-> relay board
//   Relay board powered per its own spec (typically separate 12V supply);
//   only A/B/GND go to the transceiver side.
//   120R termination if long cable / errors.
//
// NOTE: keep TX off GPIO12: GPIO12 is a strapping pin (MTDI, must be LOW
// at reset) and a transceiver pulling it HIGH bricks boot/flash
// (boot:0x33, invalid header 0xffffffff). GPIO14 is strapping-safe.
// (GPIO14 is SD CLK if you ever use an SD card.)
//
// DE/RE handling:
//   If your module is AUTO-direction (e.g. HW-0519), leave RS485_DE_PIN = -1.
//   If your module is a bare MAX485 with DE + RE pins, tie DE+RE together to
//   a free GPIO (e.g. GPIO15, NOT GPIO12) and set RS485_DE_PIN to it.
//
// USB debug console = Serial @115200 on /dev/cu.usbserial-1440.
// Type in the monitor: 1..8 = click that relay once, A = all on,
// O (or 0) = all off, S = read + print state. The loop keeps running.

#include <Arduino.h>
#include "relay_board.h"

// ---------- config ----------
#define RS485_TX_PIN 14
#define RS485_RX_PIN 13
#define RS485_BAUD RELAY_BOARD_BAUD // 9600 8N1, must match relay board
#define RS485_DE_PIN -1             // set to e.g. 15 if your MAX485 needs DE/RE driven
#define RELAY_SLAVE RELAY_BOARD_SLAVE_ADDR
#define CLICK_ON_MS 600   // how long each relay stays ON (listen for the click)
#define CLICK_OFF_MS 300  // pause after each relay turns OFF
#define NEXT_RELAY_MS 200 // pause before moving to the next relay
#define MODBUS_TIMEOUT_MS 300
#define LED_PIN 33 // ESP32CAM onboard LED (active LOW)

HardwareSerial RS485(2);
static uint8_t s_relay = 1;

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
  Serial.printf("  tx: ");
  printHex(tx, txLen);
  Serial.printf(" rx(%u): ", (unsigned)n);
  printHex(rx, n);
  Serial.println();
  return relay_board_is_write_echo(tx, rx, n);
}

// Function 0x01: read all 8 coils into *bitmap. True on valid reply.
static bool readRelays(uint8_t *bitmap) {
  uint8_t tx[RELAY_BOARD_READ_REQ_LEN];
  uint8_t rx[16];
  size_t txLen = relay_board_build_read_coils(RELAY_SLAVE, tx);
  if (txLen == 0)
    return false;
  size_t n = modbusTxn(tx, txLen, rx, sizeof(rx));
  if (!relay_board_parse_coils(rx, n, RELAY_SLAVE, bitmap)) {
    Serial.printf("  read coils FAILED rx(%u): ", (unsigned)n);
    printHex(rx, n);
    Serial.println();
    return false;
  }
  return true;
}

static void blink(int ms) {
  digitalWrite(LED_PIN, LOW); // LED on (active low)
  delay(ms);
  digitalWrite(LED_PIN, HIGH);
}

// Click one relay: ON, verify via readback, OFF. Returns true if all good.
static bool clickRelay(uint8_t relay) {
  uint8_t bm = 0;
  bool ok = true;

  Serial.printf("[R%u ON ] ", relay);
  bool wOn = setRelay(relay, true);
  bool rOn = readRelays(&bm);
  bool expectOn = rOn && relay_board_get(bm, relay);
  Serial.printf(" -> write:%s readback:0x%02X(%s) %s\n", wOn ? "OK" : "FAIL",
                bm, rOn ? "ok" : "BAD",
                (wOn && expectOn) ? "CLICK" : "*** CHECK BOARD ***");
  ok &= (wOn && expectOn);
  blink(50);
  delay(CLICK_ON_MS);

  Serial.printf("[R%u OFF] ", relay);
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
  Serial.printf("[ALL %s]\n", on ? "ON" : "OFF");
  for (uint8_t r = 1; r <= RELAY_BOARD_NUM_RELAYS; r++) {
    bool ok = setRelay(r, on);
    Serial.printf("  R%u %s:%s\n", r, on ? "ON " : "OFF", ok ? "OK" : "FAIL");
  }
}

static void handleUsb() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      line.trim();
      if (line.length() == 1) {
        char k = line.charAt(0);
        if (k >= '1' && k <= '8') {
          clickRelay((uint8_t)(k - '0'));
        } else if (k == 'A' || k == 'a') {
          allRelays(true);
        } else if (k == 'O' || k == 'o' || k == '0') {
          allRelays(false);
        } else if (k == 'S' || k == 's') {
          uint8_t bm = 0;
          if (readRelays(&bm))
            Serial.printf("[STATE] bitmap=0x%02X\n", bm);
        } else {
          Serial.println("keys: 1..8 click, A all on, O all off, S state");
        }
      } else if (line.length() > 0) {
        Serial.println("keys: 1..8 click, A all on, O all off, S state");
      }
      line = "";
    } else {
      line += c;
      if (line.length() > 16)
        line = "";
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
  // UART2 on arbitrary pins: begin(baud, config, rx, tx)
  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  Serial.println();
  Serial.println("=== ESP32CAM RELAY BOARD TEST ===");
  Serial.printf("RS485: TX=GPIO%d RX=GPIO%d baud=%d DE_PIN=%d\n", RS485_TX_PIN,
                RS485_RX_PIN, RS485_BAUD, RS485_DE_PIN);
  Serial.printf("Relay board: slave %u, %u relays, coils 0x0000-0x0007\n",
                RELAY_SLAVE, RELAY_BOARD_NUM_RELAYS);
  Serial.println("Safety: switching ALL relays OFF first.");
  allRelays(false);
  Serial.println("Loop: R1..R8 ON/OFF forever. Keys: 1..8 A O S.");
  Serial.println("===============================");
}

void loop() {
  handleUsb(); // cheap USB commands between clicks; loop never blocks on it
  bool ok = clickRelay(s_relay);
  if (!ok)
    Serial.printf("[R%u] *** no valid reply — check A/B swap, GND, "
                  "slave/baud, board power ***\n",
                  s_relay);
  s_relay = (uint8_t)(s_relay % RELAY_BOARD_NUM_RELAYS + 1);
  delay(NEXT_RELAY_MS);
}
