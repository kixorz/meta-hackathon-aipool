# ESP32CAM Combined Test — Summary

## Setup
- **ESP32CAM** (AI Thinker, `esp32cam` / PlatformIO `espressif32`, Arduino framework)
- **RS485 transceiver**, auto-direction type (e.g. HW-0519)
- **8-channel Modbus RTU relay board**, slave `1`, coils `0x0000`–`0x0007`
- **Century VGreen pump** (EVO), slave `0x15`
- Both slaves share one UART2 bus @9600 8N1; the ESP32CAM masters both
  sequentially (each slave ignores the other's frames by address/CRC)
- **PC**: USB serial adapter
  - `/dev/cu.usbserial-1440` — ESP32CAM programmer / USB debug console @115200

## Project files (`esp32-combined-test/`)
| File | Purpose |
|---|---|
| `platformio.ini` | env `esp32cam`, monitor/upload on 1440 @115200 |
| `src/main.cpp` | ESP firmware: relay sweep + pump demand/run refresh loop |
| `src/relay_board.c` | symlink → `../../src/relay_board.c` (shared driver) |
| `src/relay_board.h` | symlink → `../../include/relay_board.h` (shared driver) |
| `src/century.c` | symlink → `../../src/century.c` (shared driver) |
| `src/century.h` | symlink → `../../include/century.h` (shared driver) |
| `src/century_config.h` | symlink → `../../include/century_config.h` (shared driver) |

Protocol knowledge lives in `src/` + `include/` at the repo root; this test
only owns transport sharing and the combined sequence.

## Firmware behavior
- UART2 @9600 8N1, `TX=GPIO14`, `RX=GPIO13`, `DE_PIN=-1` (auto-direction)
- At boot: all 8 relays OFF (safety), pump probe at `0x15` (auto-detect sweep
  if silent), `set_demand_and_run(1600)`, confirm `RUNNING` + rpm in band
- Loop forever:
  - sweep relays R1–R8: `0x05` write ON → `0x01` readback verify → OFF,
    ~1.1 s per relay, ~9 s per sweep
  - every 60 s (`PUMP_REFRESH_MS`): re-issue `set_demand_and_run(1600)` and
    poll status/rpm/demand/watts. The refresh re-arms the pump's volatile
    demand register (poolpump test: it decays without refresh); rpm holds
    regardless, but the refresh keeps demand reporting 1600 too
  - two bad pump polls in a row print `*** dropout x2 ***` (keeps trying)
- Relay sweep keeps running even with no pump on the bus (relays-only mode)
- USB console keys (loop never blocks): `1`–`8` click, `A` all on, `O` all
  off, `G` toggle sweep, `R` pump run@1600, `S` pump stop, `D` demand-only,
  `P` poll both, `H` help
- Onboard LED (GPIO33) blinks on every relay write

## Wiring
```
ESP GPIO14 -> DI (driver input)      ESP GPIO13 -> RO (receiver input)
ESP GND    -> GND (required)         A <-> A, B <-> B to relay board AND pump (daisy-chain)
Relay board powered per its own spec (separate supply); only A/B/GND shared.
GPIO12: left floating. GPIO12 is a strapping pin (MTDI) — see rs485 test.
```

## How to re-run
```sh
# watch ESP console (1440 @115200); opening the port resets the board
python3 -c "import serial,time; s=serial.Serial('/dev/cu.usbserial-1440',115200,timeout=.2); t0=time.time(); [print(s.read(256).decode(errors='replace'),end='') for _ in iter(int,1) if time.time()-t0 < 200]"

# rebuild / reflash (safe with all wires connected: TX is on GPIO14, not GPIO12)
pio run -t upload
```

## Test results (2026-10-31, both slaves live on the bus)
| Test | Result |
|---|---|
| Flash `pio run -t upload` | SUCCESS, clean `boot:0x13`, no GPIO12 issue |
| Boot safety + pump probe | All relays OFF (8/8 echoed); pump answered at `0x15` immediately (already running 1615 rpm), `set_demand_and_run(1600)` ok, confirmed 1606 rpm |
| Relay sweep (200 s, ~17 sweeps) | Every ON/OFF echoed, every readback matched (`0x01`…`0x80`, `0x00`). No cross-talk with pump frames |
| Pump refresh 60 s x3 | Each `running rpm=1599–1601 demand=1600 watts=160`. Demand register stays at 1600 under 60 s refresh (vs. decaying to 1350 after 60 s silence in the poolpump test) |
| Conclusion | One ESP32CAM masters both slaves on one pair with no interference; 60 s `demand+run` refresh holds pump at ~1600 rpm indefinitely while relays click |
