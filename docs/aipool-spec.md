# AIPool spec (v1 — HTTP-only, single source of truth)

Numbers live here. Agents in `.opencode/agents/` must read this per task, never hardcode their own.

## Modes
- Logical modes: `FILTER` | `BACKWASH` only. No rinse. `moving` / `fault` are transient reports, not commands.
- Boot / comm-loss / reboot mid-run fail-safe: `FILTER`, all drives OFF.

## Valve actuator (3 relays, `relay_board` slave 1, 9600 8N1)
- Map: R1=MODE (direction select), R2=OPEN drive, R3=CLOSE drive.
- Sequence for `valve_goto()`: drives OFF → require `pump_stopped_ok` + **15 s pre-hold** (pump_stop → valve_move only, no post-hold, non-blocking) → set MODE → 200 ms settle → drive → `0x01` echo verify → drives OFF → report stable.
- Constraints: never OPEN+CLOSE together (break-before-make); still take the 15 s once if pump was already stopped.

## Pump (Century VGreen, addr `0x15`, shared UART2: TX GPIO14 / RX GPIO13, 9600 8N1)
- One bus mutex shared with relay board. `set_demand_and_run()` + 60 s refresh (volatile demand register). Software policy only: `pump_stop_for_valve()`, dropout retry → fault.
- Keep TX off GPIO12 (strapping). USB console 115200.

## Pressure gauge (VGA)
- Capture: OV2640 `VGA 640x480` grayscale (307 kpx, under 400 kpx guard). Scratch: `gauge_scratch_size(640,480)` (~1 MB) in PSRAM via `ps_malloc`, reused.
- Pipeline: `gauge_read_gray + hint + gauge_tracker_t` median-of-9, `GAUGE_YIELD()` feed, ~1 Hz. Defaults: `zero 131.0 / max 46.5 deg`, `0–60 psi`. Report `psi, conf_sigma, support, blur` per read.

## Provisioning (camera-reads-QR)
- QR payload: `WIFI:T:WPA;S:<ssid>;P:<pass>;;`. NVS stores ssid/pass/hostname (`aipool`).
- Boot: try NVS → else QR-scan mode (~15 s, LED blink) → connect → fallback AP `AIPool-XXXX`. No broker fields in v1.

## Network + HTTP API (polling only, no SSE/MQTT)
- mDNS `aipool.local`. Poll `GET /api/state` @1 Hz:
  `{"psi","rpm","watts","mode":"filter|backwash|moving|fault","valve","pump","fault","threshold_psi","backwash_min"}`.
- `POST /api/cmd {"mode":"filter|backwash","threshold_psi":float,"backwash_min":<=10,"rpm":600..3450}` — validated, NVS-persisted. Thresholds adjustable here.
- `GET /gauge` single VGA JPEG snapshot (no stream), `GET /health`.
- Backwash: `PUMP_STOP → 15 s → VALVE_MOVE → BACKWASH_RUN(≤10 min hard kill) → PUMP_STOP → 15 s → VALVE_MOVE → FILTER`. 10-min timer starts at run entry.

## Drivers (don't fork)
- `src/gauge.c`, `src/relay_board.c`, `src/century.c` + `include/` are pure C. Fix at root; `tests/*/src` symlinks only. Reference bus proof: `tests/esp32-combined-test/summary.md`.
