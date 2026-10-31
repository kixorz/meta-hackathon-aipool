# ESP32CAM Relay Board Test — Summary

## Setup
- **ESP32CAM** (AI Thinker, `esp32cam` / PlatformIO `espressif32`, Arduino framework)
- **RS485 transceiver**, auto-direction type (e.g. HW-0519)
- **8-channel Modbus RTU relay board**, slave `1`, 9600 baud 8N1,
  coils `0x0000`–`0x0007` = relays 1–8
- **PC**: USB serial adapter
  - `/dev/cu.usbserial-1440` — ESP32CAM programmer / USB debug console @115200

## Project files (`esp32-relays-test/`)
| File | Purpose |
|---|---|
| `platformio.ini` | env `esp32cam`, monitor/upload on 1440 @115200 |
| `src/main.cpp` | ESP firmware: Modbus RTU loop clicking relays 1–8 |
| `src/relay_board.c` | symlink → `../../src/relay_board.c` (shared driver, compiled as-is) |
| `src/relay_board.h` | symlink → `../../src/relay_board.h` (shared driver) |

The protocol knowledge (function codes `0x01`/`0x02`/`0x05`, CRC-16/Modbus,
coil map, known-good frame vectors) lives in `src/relay_board.*` at the repo
root. This test only owns transport (half-duplex UART2 send/reply with
resync + inter-byte-silence framing) and the click sequence. Fix protocol
bugs in `src/`, not here.

## Firmware behavior
- UART2 @9600 8N1, `TX=GPIO14`, `RX=GPIO13`, `DE_PIN=-1` (auto-direction)
- At boot: writes all 8 relays OFF (safety), then loops forever:
  `R1 ON → readback verify → R1 OFF → R2 … → R8 → repeat`
- Each step uses `0x05 Write Single Coil` and requires the board's echo
  (`tx == rx`, CRC OK); after each write it issues `0x01 Read Coils` and
  checks the relay bit in the bitmap. Mismatches print `*** CHECK BOARD ***`
- Timings (`#define` in `main.cpp`): `CLICK_ON_MS=600`, `CLICK_OFF_MS=300`,
  `NEXT_RELAY_MS=200`, `MODBUS_TIMEOUT_MS=300`
- USB console keys (loop keeps running): `1`–`8` click once, `A` all on,
  `O`/`0` all off, `S` read + print bitmap
- Onboard LED (GPIO33) blinks on every relay write

## Wiring
```
ESP GPIO14 -> DI (driver input)      ESP GPIO13 -> RO (receiver output)
ESP GND    -> GND (required)         A <-> A, B <-> B to the relay board
Relay board powered per its own spec (separate supply); only A/B/GND shared.
GPIO12: left floating. GPIO12 is a strapping pin (MTDI) — see rs485 test.
```

## How to re-run
```sh
# watch ESP console (1440 @115200)
pio device monitor

# rebuild / reflash (safe with all wires connected: TX is on GPIO14, not GPIO12)
pio run -t upload
```

## Expected console output
```
=== ESP32CAM RELAY BOARD TEST ===
RS485: TX=GPIO14 RX=GPIO13 baud=9600 DE_PIN=-1
Relay board: slave 1, 8 relays, coils 0x0000-0x0007
Safety: switching ALL relays OFF first.
[ALL OFF]
  ...
Loop: R1..R8 ON/OFF forever. Keys: 1..8 A O S.
===============================
[R1 ON ]   tx: 01 05 00 00 FF 00 8C 3A  rx(8): 01 05 00 00 FF 00 8C 3A  -> write:OK readback:0x01(ok) CLICK
[R1 OFF]  tx: 01 05 00 00 00 00 CD CA  rx(8): 01 05 00 00 00 00 CD CA  -> write:OK readback:0x00(ok) CLACK
...
```

No reply at all usually means: A/B swapped, missing common GND, wrong
slave/baud on the board, or the relay board has no power.

## Test results (2026-10-31, live board on the bus)
| Test | Result |
|---|---|
| Flash `pio run -t upload` | SUCCESS, clean `boot:0x13`, no GPIO12 issue (TX on GPIO14) |
| Boot safety sweep | All 8 relays OFF, every `0x05` echoed |
| Click loop R1–R7 observed | Every ON/OFF echoed, every `0x01` readback matched (`0x01`, `0x02`, `0x04` …), `CLICK`/`CLACK` throughout |
| Conclusion | ESP32CAM → transceiver → relay board path fully verified |
