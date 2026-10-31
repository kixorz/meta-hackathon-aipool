// ESP32CAM RS485 bus test
// TX = GPIO14 (to DI on transceiver), RX = GPIO13 (from RO on transceiver)
// NOTE: TX was moved off GPIO12: GPIO12 is a strapping pin (MTDI, must be LOW
// at reset) and the transceiver pulls it HIGH, which bricks boot/flash.
// GPIO14 is strapping-safe. (GPIO14 is SD CLK if you ever use an SD card.)
// USB debug console = Serial @115200 on /dev/cu.usbserial-1440 (flash/programmer port)
// RS485 bus        = Serial2 @RS485_BAUD on /dev/cu.usbserial-1430 side (via RS485 dongle)
//
// Wiring (MAX485 style):
//   ESP GPIO14 -> DI (driver input)
//   ESP GPIO13 -> RO (receiver output)
//   ESP GND    -> GND (MUST share ground with bus, even though RS485 is differential)
//   A <-> A, B <-> B across ESP transceiver <-> USB-RS485 dongle
//   120R termination if long cable / errors (many dongles already have it).
//
// DE/RE handling:
//   If your module is AUTO-direction (e.g. HW-0519, TTL-to-RS485 with 1206 chip),
//   leave RS485_DE_PIN = -1.
//   If your module is a bare MAX485 with DE + RE pins, tie DE+RE together to
//   a free GPIO (e.g. GPIO15, NOT GPIO12) and set RS485_DE_PIN to it.
//   (GPIO14 is now the TX pin, so it can't double as DE.)
//
// GPIO12 is a strapping pin (MTDI): it must be LOW at reset or the ESP32
// selects 1.8V flash voltage and won't boot/flash. TX lives on GPIO14 now,
// so nothing on the RS485 side can disturb boot — no disconnect dance needed.

#include <Arduino.h>

// ---------- config ----------
#define RS485_TX_PIN 14
#define RS485_RX_PIN 13
#define RS485_BAUD 9600
#define RS485_DE_PIN -1   // set to e.g. 14 if your MAX485 needs DE/RE driven, else -1
#define HELLO_PERIOD_MS 2000
#define LED_PIN 33        // ESP32CAM onboard LED (active LOW)

HardwareSerial RS485(2);
uint32_t counter = 0;
unsigned long lastHello = 0;
String rxLine;

static void rs485TxEnable(bool tx) {
#if RS485_DE_PIN >= 0
  digitalWrite(RS485_DE_PIN, tx ? HIGH : LOW);
  if (tx) delayMicroseconds(200);
#endif
}

static void rs485Send(const String &s) {
  rs485TxEnable(true);
  RS485.print(s);
  RS485.flush();  // wait for shift register to empty
  delay(2);       // guard time for slow transceivers
  rs485TxEnable(false);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED off (active low)

#if RS485_DE_PIN >= 0
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);  // start in receive mode
#endif

  Serial.begin(115200);
  delay(500);
  // UART2 on arbitrary pins: begin(baud, config, rx, tx)
  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  Serial.println();
  Serial.println("=== ESP32CAM RS485 TEST ===");
  Serial.printf("RS485: TX=GPIO%d RX=GPIO%d baud=%d DE_PIN=%d\n",
                RS485_TX_PIN, RS485_RX_PIN, RS485_BAUD, RS485_DE_PIN);
  Serial.println("Commands on USB serial forwarded to RS485.");
  Serial.println("Type PING on either side, other side replies PONG.");
  Serial.println("===========================");
}

void loop() {
  unsigned long now = millis();

  // 1) periodic beacon out on RS485
  if (now - lastHello >= HELLO_PERIOD_MS) {
    lastHello = now;
    counter++;
    // simple checksum so PC can verify integrity
    uint8_t cks = 0;
    String payload = "HELLO " + String(counter) + " FROM ESP32CAM";
    for (size_t i = 0; i < payload.length(); i++) cks ^= payload[i];
    String msg = "ESP:" + payload + " CK=" + String(cks, HEX) + "\n";

    rs485Send(msg);
    Serial.printf("[TX-RS485] %s", msg.c_str());
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
  }

  // 2) RS485 -> USB debug + auto-reply
  while (RS485.available()) {
    char c = (char)RS485.read();
    Serial.write(c);  // live mirror so you see bus traffic on /dev/cu.usbserial-1440

    if (c == '\n' || c == '\r') {
      rxLine.trim();
      if (rxLine.length() > 0) {
        Serial.printf("\n[RX-RS485] \"%s\"\n", rxLine.c_str());
        digitalWrite(LED_PIN, LOW);
        delay(20);
        digitalWrite(LED_PIN, HIGH);
        if (rxLine == "PING" || rxLine.endsWith("PING")) {
          rs485Send("PONG from ESP32CAM\n");
          Serial.println("[TX-RS485] PONG from ESP32CAM");
        } else {
          rs485Send("ACK:" + rxLine + "\n");
        }
      }
      rxLine = "";
    } else {
      rxLine += c;
      if (rxLine.length() > 128) rxLine = "";  // resync on garbage
    }
  }

  // 3) USB serial -> RS485 (lets you type from PlatformIO monitor to test TX path)
  if (Serial.available()) {
    rs485TxEnable(true);
    while (Serial.available()) {
      RS485.write((uint8_t)Serial.read());
    }
    RS485.flush();
    delay(2);
    rs485TxEnable(false);
  }
  delay(1);
}
