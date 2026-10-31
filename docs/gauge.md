# gauge — portable analog-gauge reader

Pure C99 core: one C file + one header, no malloc, no file I/O, no platform
headers. Give it an 8-bit gray frame, get back dial geometry, needle angle,
and calibrated value.

```
include/gauge.h   public API (Doxygen-commented, extern "C")
src/gauge.c       implementation
docs/gauge.md     this file
```

## Model

```
gray[w*h] → find_dial → 360° ray scan → ±4° refine → tip/tail vote → value
```

1. **Dial detection** (`find_dial_f`): downsample ×2, Sobel-ish gradients,
   vote along ±normal for center, radius histogram for `R`. Global search
   needs peak ≥ 40 votes; tracked band search (annulus ±55% around hint)
   needs ≥ 12. `support = peak / voting_edges` measures circle evidence.
2. **Needle scan**: mean side-minus-center contrast along each ray
   (`R*0.25 → R*0.95`), dual offset (1.5px thin + `R/14` thick, clamped
   3–6px), outer-half weighting (tip counts more). `reach` = fraction of
   outer samples with contrast > 3.
3. **Tip/tail disambiguation**: raw max vs opposite (+180°), each refined
   ±4° @ 0.5°. Primary rule = hub darkness (stub root ~80 gray vs faint
   hand ~200+; 25-gray margin), fallbacks = dark-run length (+0.15 R),
   outer reach (+0.30 with ≥0.6× score). `calib.flip` forces the alternate.
4. **Value**: clockwise sweep `zero_ang → max_ang` (fallback 270° if
   degenerate); outside sweep clamps to nearer endpoint.
5. **Quality**: `conf_sigma = (tip − mean)/std` over 360 scores (< 2 weak),
   `blur` = Laplacian variance (< 20 blurry, < 60 soft).
6. **Tracker** (`gauge_tracker_t`, optional): geometry re-lock on jumps
   (> 25px / 35% R), hysteresis toward lock (12° bias), 15°/reading slew
   limit, weak-frame unlock (support < 0.010 ×3), median-of-9 over values.

## Conventions

* Image: x right, y **down**. Angles deg: `0=E, 90=S, 180=W, 270=N`,
  clockwise positive. Same for `zero_ang`, `max_ang`, `angle_deg`.
* Value maps **clockwise** from `zero_ang` to `max_ang`.
* Input minimum 160×120 (`GAUGE_ERR_SIZE` below).

## Usage

```c
#include "gauge.h"

static gauge_tracker_t g_trk;
static uint8_t scratch[/* gauge_scratch_size(W,H) */]; // or PSRAM malloc

gauge_calib_t cal = { .zero_ang = 131.0f, .max_ang = 46.5f,
                      .vmin = 0, .vmax = 60, .flip = 0 };
gauge_tracker_init(&g_trk);

// per frame:
gauge_hint_t hint = { .enable = g_trk.init,
                      .gcx = g_trk.cx, .gcy = g_trk.cy, .gR = g_trk.R };
gauge_result_t r;
if (gauge_read_gray(gray, w, h, &cal, &hint,
                    scratch, sizeof scratch, &r) == GAUGE_OK) {
  gauge_stab_t s;
  gauge_tracker_update(&g_trk, &r, &cal, hint.enable, &s);
  printf("angle=%.1f value=%.1f median=%.1f conf=%.2f\n",
         s.stab_angle, s.stab_value, s.median_value, r.conf_sigma);
}
```

* `hint = NULL` or `.enable = 0` = global search (first frame, overlay).
* `?lock=0` diagnostic now bypasses hysteresis/slew **and** the median
  (pure single-frame `raw_angle`/`value`); previously the median-of-9 still
  applied with the lock off. Locked mode (`?lock=1`, default) is unchanged.
* `scratch` must be ≥ `gauge_scratch_size(w,h)`; reuse across frames of the
  same size. Contents are throwaway.
* Single-frame diagnostic: skip the tracker, use `r.angle_deg / r.value`.
* Reset on scene change or `?reset=1`: `gauge_tracker_reset(&trk)`.
* Cooperative yield: compile with `-DGAUGE_YIELD=my_watchdog_feed` to hook
  the vote loop; default no-op.

## Memory

Worst case ≈ `17 × (w/2·h/2) + 360×4 + hist` bytes. VGA 640×480 →
downsampled 320×240 = 57 600 px → ~1 MB. The on-device caller (`/gauge`)
guards frames to ≤ 400 kpx (≈VGA); QVGA/host tests are far smaller.
Size the scratch buffer with `gauge_scratch_size()` — never hand-roll.

## Calibration

1. Fix camera, capture BMP (`/bmp`), run `tickprof` to find tick angles.
2. Set `zero_ang` = 0-tick angle, `max_ang` = max-tick angle
   (defaults 131.0 / 46.5 for the 0–60 psi rig, 275.5° sweep).
3. Set `vmin/vmax` to gauge range. Verify with overlay (`/overlay`):
   green = picked tip, yellow = alt, blue = dial, red = hub.
4. If the wrong end wins persistently, pass `flip=1` once to confirm,
   then fix lighting/mount — flip is an override, not a calibration.

## Errors

| code | meaning |
|---|---|
| `GAUGE_OK` | success |
| `GAUGE_ERR_ARG` | NULL pointer |
| `GAUGE_ERR_SIZE` | image < 160×120 |
| `GAUGE_ERR_NODIAL` | no dial circle found |
| `GAUGE_ERR_NOMEM` | scratch < `gauge_scratch_size(w,h)` |

## Porting

* C99 + `math.h` (`sqrtf/cosf/sinf`) only. Float-only; no double promotion
  in hot paths. Reentrant for distinct `(scratch, out, tracker)` instances.
* Replaces legacy `src/gauge.cpp` (Arduino/PSRAM/`delay`) and the
  file-static lock/median in `app_httpd.cpp`; behavior preserved
  (0.75/0.25 lock blend, same thresholds), now explicit and testable.
* Host check: `gcc -std=c99 -O2 -Wall -Wextra -Iinclude -fsyntax-only src/gauge.c`.
