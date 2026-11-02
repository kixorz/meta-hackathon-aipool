# ESP32CAM QR Scan Test — Summary

## Setup
- **ESP32CAM** (AI Thinker, `esp32cam` / PlatformIO `espressif32`, Arduino framework)
- **Printed QR code** (any payload; a `WIFI:T:...;S:...;P:...;;` code exercises
  the provisioning path), held 15–25 cm in front of the lens, flat, lit,
  filling 1/4+ of the frame
- **PC**: USB serial adapter
  - `/dev/cu.usbserial-1440` — ESP32CAM programmer / USB debug console @115200
- No RS485 wiring needed for this test (camera only)

## Project files (`esp32-qrscan-test/`)
| File | Purpose |
|---|---|
| `platformio.ini` | env `esp32cam`, monitor/upload on 1440 @115200 |
| `src/main.cpp` | ESP firmware: camera init + self-test + scan loop (owns transport/diagnostics only) |
| `src/qrscan.c` | symlink → `../../src/qrscan.c` (shared driver, compiled as-is) |
| `src/qrscan.h` | symlink → `../../include/qrscan.h` (shared driver) |
| `src/quirc/*` | symlinks → `../../src/quirc/*` (vendored quirc backend: `quirc.c`, `decode.c`, `identify.c`, `version_db.c` + headers, MIT) |
| `src/openmv/*` | symlinks → `../../src/openmv/*` (vendored OpenMV helpers required by `identify.c`: `collections.c/.h`, `fmath.h`, MIT) |

The decode knowledge (grayscale auto-levels, decode + mirror retry, half-res
and 1px-jiggle second chances, candidate diagnostics) lives in
`src/qrscan.c` at the repo root. This test only owns the camera setup, the
capture loop, and the serial diagnostics. Fix decode bugs in `src/`, not here.

## Firmware behavior
- Boot runs an error-path self-test (no camera needed):
  `qrscan_decode(NULL)` → `-6`, 10x10 image → `-6`, flat-gray 64x64 →
  `-1` with zero candidates, plus `qrscan_strerror` coverage and a
  zero-init `qrscan_free` double-call. Prints `ALL SELF-TESTS PASSED` or
  `*** SELF-TEST FAILED ***`.
- Camera: OV2640 `PIXFORMAT_GRAYSCALE`, `FRAMESIZE_VGA` (640x480) in PSRAM,
  QR-tuned DSP (`contrast=2`, brightness/AE neutral — same as the
  provisioning scan).
- Continuous loop (default ON): capture → `qrscan_decode_verbose` → one
  `[QR] fN WxH full=A half=B rc=C (str)` line per frame, per-candidate
  geometry lines, and a `DECODED v.. ecc=.. mask=.. dim=.. len=.. data=".."`
  line plus LED blink on success. `full`/`half` pack as `full*100+half`.
- USB console keys (loop never blocks): `C` toggle continuous, `S` single
  snapshot, `D` snapshot + ascii thumbnail (`qrscan_dump_image_ascii`),
  `F` full-res grayscale frame as hex between `pgm begin/end` markers
  (host script reassembles to PGM/PNG — shows exactly what quirc sees),
  `L` camera flash LED (GPIO4) on/off (ON at boot), `V` sensor vflip
  on/off, `M` sensor hmirror on/off (`V+M` = 180-degree rotation; this
  rig's module is mounted upside-down so both default ON — decode itself
  is rotation-invariant),
  `H` help. If camera init fails the test drops to self-test + manual
  snapshots.
- Workspace: `static qr_workspace_t` (~39KB, never the stack).

## How to re-run
```sh
pio device monitor

pio run -t upload
```

## Expected console output
```
=== ESP32CAM QR SCAN TEST ===
Reuses src/qrscan.c + include/qrscan.h over src/quirc/.
Keys: C continuous, S snapshot, D snapshot+ascii, H help.
[SELF] null img -> -6 (bad image or no memory) OK
[SELF] tiny 10x10 -> -6 (bad image or no memory) OK
[SELF] flat 64x64 -> rc=-1 (no QR found) count=0 OK
...
[SELF] ALL SELF-TESTS PASSED
Camera VGA 640x480 gray ready. Continuous ON.
===============================
[QR] f1 640x480 full=0 half=0 rc=-1 (no QR found)
[QR] hint: no QR found — hold code 15-25cm in front of lens, fill 1/4+ of frame, flat, lit
...
[QR] f12 640x480 full=1 half=0 rc=0 (ok)
[QR]   cand0 size=29 err=0 plen=32 corners=(180,120)(320,118)(322,258)(182,260)
[QR] DECODED v2 ecc=M mask=1 dim=29 len=32 data="WIFI:T:WPA;S:MyNet;P:secret;;"
```

No decode at all usually means: code too small/far (aim 250–350 px wide),
glare or backlight washing out blacks, motion blur (brace hands 2–3 s), or
the lens protective film still on.

## Test results (2026-11-01, live on ESP32CAM via /dev/cu.usbserial-1430)
| Test | Result |
|---|---|
| `pio run` (esp32cam) | SUCCESS, RAM 26.3%, Flash 11.7% |
| `pio run -t upload` | SUCCESS, clean `boot:0x13`, `POWERON_RESET`, no GPIO12 issue |
| Boot self-test | ALL SELF-TESTS PASSED: `free(zero-init) x2`, `null -> -6`, `tiny -> -6`, `flat 64x64 -> -1 count=0`, full `strerror` coverage |
| Camera init | VGA 640x480 gray ready, first try |
| Continuous scan (20 frames, no QR in view) | Stable ~1 fps, `full=0 half=0 rc=-1` throughout, hints every 5th frame, zero resets / zero WDT |
| `F` full-frame PGM dump | 307200/307200 bytes reassembled to PGM+PNG on host; viewed: QR present bottom-right but cut off at the frame edge (see Notes) |
| Flash LED (`L`, ON at boot) | Night-and-day: dark frame -> bright high-contrast frame; grid now found every frame (`full=1`, v2 29x29, stable corners) |
| Live QR decode (`DECODED ...`) | PASSED 2026-11-01: first frame after boot decoded `v3 ecc=L mask=4 dim=29 len=38` — a 38-byte `WIFI:T:WPA;S:...;P:...;;` provisioning payload (secret redacted here). Notably via the half-res second-chance path (`full=0 half=1`): full-res found no grid, the 2x-downsampled pass locked and decoded. Corners pixel-stable, near-perfect rectangle (264,93)(397,93)(396,226)(262,224) |
| Orientation (`V`/`M`, default ON/OFF) | RESOLVED: paper ground truth (missing finder = bottom-right = normal print) + template-match of the (1,1) stream (finders TL/TR/BR, BL absent = mirrored) proved the module reads out vflip-native. vflip-only (1,0) restores the true image — verified by template-match (TL 0.74 / TR 0.83 / BL 0.70 present, BR 0.10 absent = normal). Driver accepts both flips (`rc=0/0`) |

## Notes
- The self-test initially failed in a way worth recording: a local
  `qr_workspace_t scratch` (~39KB) on the 8KB loopTask stack moved SP ~40KB
  out of bounds (confirmed via `objdump`: `sp -= 40048`), wedging the UART
  and tripping `TG1WDT_SYS_RESET` in a boot loop with no crash dump. Fix:
  reuse the static `s_ws` for the zero-init double-free check (a second
  39KB static overflows `dram0_0_seg` at link time — the DRAM budget fits
  exactly one workspace + the 4KB blank frame). The `qrscan.h` "NEVER stack"
  warning is load-bearing; this test now guards it.
- `pio device monitor` needs a real TTY (miniterm `termios` fails in pipes);
  capture with `python3 -c` + pyserial (or `screen -L`) instead.
