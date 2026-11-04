# ESP32CAM Valve Test — Summary

## Setup
- **ESP32CAM** (AI Thinker, `esp32cam` / PlatformIO `espressif32`, Arduino framework)
- **RS485 transceiver**, auto-direction type (e.g. HW-0519)
- **8-channel Modbus RTU relay board**, slave `1`, 9600 baud 8N1,
  relays `1`–`3` wired as the pool valve actuator, bench-verified latched
  states: `FILTER` = `R2` ON, `BACKWASH` = `R1`+`R3` ON. Pump is NEVER
  touched by this test.
- **PC**: USB serial adapter
  - `/dev/cu.usbserial-1440` — ESP32CAM programmer / USB debug console @115200

## Project files (`esp32-valve-test/`)
| File | Purpose |
|---|---|
| `platformio.ini` | env `esp32cam`, monitor/upload on 1440 @115200 |
| `src/main.cpp` | ESP firmware: ~60 s FILTER→BACKWASH→FILTER with parked holds, then manual |
| `src/relay_board.c` | symlink → `../../src/relay_board.c` (shared driver, compiled as-is) |
| `src/relay_board.h` | symlink → `../../include/relay_board.h` (shared driver) |

The valve map (FILTER = R2 latched, BACKWASH = R1+R3 latched, ~20 s travel)
is bench truth established by this test and now matches
`firmware/app/src/valve.h` + `docs/aipool-spec.md` — the app driver was
updated to the same latched-position model (an earlier pulse model never
moved this hardware).

## Firmware behavior
- UART2 @9600 8N1, `TX=GPIO14`, `RX=GPIO13`, `DE_PIN=-1` (auto-direction)
- At boot: all 8 relays OFF (safety), then after a 10 s abort window (any
  key = manual mode) the auto sequence runs once (~60 s):
  `FILTER` (break-before-make, latch R2, hold ~20 s) →
  `BACKWASH` (break-before-make, latch R1+R3, hold ~20 s) →
  `FILTER` (break-before-make, latch R2, hold ~20 s) →
  all valve relays OFF (end safe state).
- The valve needs ~20 s to travel between positions, so each hold doubles
  as the travel window: the next position is commanded only after the
  previous one had time to arrive. Relay readback confirms the latch, not
  arrival — there is no position feedback on this hardware; arrival is
  inferred from the travel window (listen for the actuator to stop).
- Transitions are break-before-make (R1/R2/R3 all OFF first); holds poll
  the `0x01` readback every 2 s and require the exact latched bitmap.
  Each phase prints PASS/FAIL plus a final
  `[VERDICT] FILTER-1:.. BACKWASH:.. FILTER-2:..` line, then manual mode.
- Expected bitmaps: FILTER phases `0x02` (R2 latched), BACKWASH phase
  `0x05` (R1+R3 latched), end state `0x00` (all off).
- USB console keys (loop never blocks): `F` latch filter, `B` latch
  backwash, `O` all relays off, `S` read + print bitmap, `R` re-run auto,
  `H` help. Pump is never driven — run only with the pump stopped.
- Onboard LED (GPIO33) blinks on every successful relay write.

## Wiring
```
ESP GPIO14 -> DI (driver input)      ESP GPIO13 -> RO (receiver output)
ESP GND    -> GND (required)         A <-> A, B <-> B to the relay board
Relay board powered per its own spec (separate 12V supply); only A/B/GND shared.
GPIO12: left floating (strapping pin MTDI — see rs485 test).
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
=== ESP32CAM VALVE TEST (relay board, ~60 s) ===
RS485: TX=GPIO14 RX=GPIO13 baud=9600 DE_PIN=-1
Map: FILTER=R2 on, BACKWASH=R1+R3 on (latched states).
Safety: all relays OFF first. Pump is NEVER touched here.
...
[t+10s]=== AUTO-1: -> FILTER (R2 on) ===
[t+10s][AUTO-1] latch FILTER: R2 ON...
[t+10s][AUTO-1] holding 0x02 for 20 s (readback every 2000 ms)...
[t+10s][AUTO-1] bitmap=0x02 expect=0x02 MATCH
...
[t+31s][AUTO-1] hold HELD
[t+31s]=== AUTO-1: FILTER PASS ===
[t+31s]=== AUTO-2: -> BACKWASH (R1+R3 on) ===
[t+31s][AUTO-2] latch BACKWASH: R1 ON + R3 ON...
[t+31s][AUTO-2] holding 0x05 for 20 s (readback every 2000 ms)...
[t+31s][AUTO-2] bitmap=0x05 expect=0x05 MATCH
...
[t+52s][AUTO-2] hold HELD
[t+52s]=== AUTO-2: BACKWASH PASS ===
...
[t+73s]=== AUTO-3: FILTER PASS ===
[t+73s][SAFE] end state: all valve relays OFF
[t+73s][VERDICT] FILTER-1:PASS BACKWASH:PASS FILTER-2:PASS => ALL PASS
Manual mode. Keys: F B O S R H. (Pump is never touched.)
```

## Test results (2026-11-03, live on ESP32CAM + relay board @1440)
| Test | Result |
|---|---|
| `pio run -t upload` | SUCCESS, clean `boot:0x13`, no GPIO12 issue |
| Boot safety sweep | All 8 relays OFF, every `0x05` echoed |
| Auto FILTER→BACKWASH→FILTER (~63 s) | ALL PASS: every latch echoed, every 2 s readback matched (`0x02`/`0x05`), holds HELD, end state all OFF, zero WDT resets |
| Manual keys (F/B/O/S/R) | F/B run the same phase functions as auto (verified path); O/S/R not individually exercised live |
