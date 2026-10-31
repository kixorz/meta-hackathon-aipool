# ESP32CAM Pool Pump Test — Summary

## Setup
- **ESP32CAM** (AI Thinker, `esp32cam` / PlatformIO `espressif32`, Arduino framework)
- **RS485 transceiver**, auto-direction type (e.g. HW-0519)
- **Century VGreen pump** (EVO), slave `0x15`, 9600 baud 8N1
- **PC**: USB serial adapter
  - `/dev/cu.usbserial-1440` — ESP32CAM programmer / USB debug console @115200

## Project files (`esp32-poolpump-test/`)
| File | Purpose |
|---|---|
| `platformio.ini` | env `esp32cam`, monitor/upload on 1440 @115200 |
| `src/main.cpp` | ESP firmware: auto experiment (probe → soak → remind) + USB manual control |
| `src/century.c` | symlink → `../../src/century.c` (shared driver, compiled as-is) |
| `src/century.h` | symlink → `../../include/century.h` (shared driver) |
| `src/century_config.h` | symlink → `../../include/century_config.h` (shared driver) |

The protocol knowledge (func codes `0x41`–`0x45`/`0x64`/`0x65`, CRC-16/Modbus,
sensor pages, EVO quirk frame) lives in `src/century.c` + `include/century.h`
at the repo root. This test only owns the UART2 transport and the experiment
sequence. Fix protocol bugs in `src/` + `include/`, not here.

## Firmware behavior
- UART2 @9600 8N1, `TX=GPIO14`, `RX=GPIO13`, `DE_PIN=-1` (auto-direction)
- At boot: status probe at `0x15`, full `auto_detect` sweep if silent
- 10 s abort window (any key → manual), then the auto experiment:
  - **PROBE**: `set_demand_and_run(1600)`, confirm `RUNNING` + rpm in band
  - **SOAK**: 12x read-only polls (status/rpm/demand/watts) every 10 s,
    then silent gaps with **zero bus TX** (60/120/300 s), one poll after each
  - **REMIND** (only on dropout): re-issue `RUN` every 30 s x6, check it holds
  - Verdict printed, then manual mode
- USB console keys (loop never blocks): `R` run@1600, `S` stop, `D` demand-only,
  `P` poll, `A` detect, `X` manual, `H` help
- Driver `TX`/`RX` frames echo to console at `[C D]` level

## Wiring
```
ESP GPIO14 -> DI (driver input)      ESP GPIO13 -> RO (receiver input)
ESP GND    -> GND (required)         A <-> A, B <-> B to the pump
GPIO12: left floating. GPIO12 is a strapping pin (MTDI) — see rs485 test.
```

## How to re-run
```sh
# watch ESP console (1440 @115200); opening the port resets the board,
# which restarts the experiment from t+0
python3 -c "import serial,time; s=serial.Serial('/dev/cu.usbserial-1440',115200,timeout=.2); t0=time.time(); [print(s.read(256).decode(errors='replace'),end='') for _ in iter(int,1) if time.time()-t0 < 700]"
# or: pio device monitor (needs a TTY)

# rebuild / reflash (safe with all wires connected: TX is on GPIO14, not GPIO12)
pio run -t upload
```

## Test results (2026-10-31, live pump on the bus)
| Test | Result |
|---|---|
| Flash `pio run -t upload` | SUCCESS, clean `boot:0x13`, no GPIO12 issue |
| Probe at `0x15` @9600 | Pump answered immediately (was already running 847 rpm, demand 1350 — leftover state) |
| `set_demand_and_run(1600)` | `ok`: demand echo `00 19`, run acked; ramp 1475 → 1600 within ~10 s, 98 → 160 W |
| SOAK read-only polls (12x) | All `running`, rpm 1598–1611, demand 1600, watts ~160 |
| Silent gap 60 s, zero TX | Pump held 1601 rpm. **Notable: demand register read back 1350 while rpm stayed 1601** — the volatile demand value appears to decay to a stored value without refresh, but the motor holds speed |
| Silent gaps 120 s / 300 s + verdict | Not yet observed (capture aborted mid-gap); re-run the 700 s listen above to complete |

## Open questions
- Does rpm eventually decay after longer silence, or does the pump hold 1600
  indefinitely with no reminders? Finish the 120/300 s gaps to answer.
- What re-arms the demand register — any addressed frame, or only `0x44`?
  If rpm holds regardless, reminders are unnecessary for this use case.
