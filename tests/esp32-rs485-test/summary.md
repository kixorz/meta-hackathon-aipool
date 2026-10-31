# ESP32CAM RS485 Bus Test — Summary

## Setup
- **ESP32CAM** (AI Thinker, `esp32cam` / PlatformIO `espressif32`, Arduino framework)
- **RS485 transceiver**, auto-direction type (e.g. HW-0519)
- **PC**: two USB serial adapters
  - `/dev/cu.usbserial-1440` — ESP32CAM programmer / USB debug console @115200
  - `/dev/cu.usbserial-1430` — USB-RS485 dongle @9600

## Project files (`esp32-rs485-test/`)
| File | Purpose |
|---|---|
| `platformio.ini` | env `esp32cam`, monitor/upload on 1440 @115200 |
| `src/main.cpp` | ESP firmware: UART2 beacons + PING/PONG + USB bridge |
| `rs485_pc_test.py` | PC listener / ping / stress tester on 1430 @9600 |

## Firmware behavior
- UART2 @9600 8N1, `TX=GPIO14`, `RX=GPIO13`, `DE_PIN=-1` (auto-direction)
- Every 2 s sends `ESP:HELLO <n> FROM ESP32CAM CK=<xor-checksum>`
- Replies `PONG from ESP32CAM` to `PING`, `ACK:<line>` to anything else
- Mirrors all RS485 traffic to the USB console; USB-typed lines forward to RS485
- Onboard LED (GPIO33) blinks on TX/RX

## Final wiring
```
ESP GPIO14 -> DI (driver input)      ESP GPIO13 -> RO (receiver output)
ESP GND    -> GND (required)         A <-> A, B <-> B to the 1430 dongle
GPIO12: left floating (see below)
```

## The GPIO12 boot quirk (diagnosed & fixed)
- **Symptom**: flash failed (`Failed to communicate with the flash chip`,
  `Packet content transfer stopped`); board boot-looped with
  `boot:0x33 … invalid header: 0xffffffff`.
- **Root cause**: GPIO12 (MTDI) is a strapping pin — HIGH at reset selects 1.8V
  flash voltage, making the flash unreadable. The transceiver's DI input pulls
  GPIO12 HIGH. A healthy boot shows `boot:0x13`. The `0xFF` reads were the flash
  misbehaving at the wrong voltage, not erased firmware.
- **Fix**: moved RS485 TX from GPIO12 → **GPIO14** (not a strapping pin, free on
  the ESP32CAM header, unused by the camera). RX on GPIO13 was always safe.
  (GPIO14 is SD CLK — only a conflict if an SD card is used. No SD + no camera
  pin conflicts in this project.)
- **Proof**: hard reset with all RS485 wires connected → `boot:0x13`, clean boot.
  A later inadvertent reset also recovered on its own. Resets, power-cycles and
  reflashes now work with everything connected — no more disconnect dance.

## Test results
| Test | Result |
|---|---|
| Beacon listen, 10 s (`--timeout 10`) | 5/5 beacons, sequential counters, checksums OK |
| Round-trip stress (`--stress 10`) | 10/10 `PONG` replies (repeated runs) |
| One-off 9/10 run | Explained: ESP beacon collided on-air with PC `PING` (garbled frame in log). Expected on half-duplex RS485 without collision avoidance, not a wiring fault. |

## How to re-run
```sh
# watch ESP console (1440 @115200)
python3 -c "import serial,time; s=serial.Serial('/dev/cu.usbserial-1440',115200,timeout=.2); ..."  # or pio device monitor from a TTY

# beacon test (ESP -> PC)
python3 rs485_pc_test.py --timeout 10

# round-trip stress (both directions)
python3 rs485_pc_test.py --stress 10

# rebuild / reflash (safe with all wires connected now)
pio run -t upload
```
