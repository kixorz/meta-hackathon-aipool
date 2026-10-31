/* gauge.c — portable analog-gauge reader core.
 *
 * C99, float-only, no file I/O, no malloc, no platform headers.
 * Ported from src/gauge.cpp (ESP32 on-device version); the algorithm is
 * unchanged, only allocation (caller scratch) and stabilization (explicit
 * gauge_tracker_t) were decoupled for reuse.
 *
 * Optional cooperative-yield hook: define GAUGE_YIELD() before including,
 * or -DGAUGE_YIELD=my_yield, to run e.g. a watchdog feed inside the vote
 * loop. Default is a no-op.
 */
#include "gauge.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef GAUGE_YIELD
#define GAUGE_YIELD() ((void)0)
#endif

/* ---------- small utils ---------- */

static float deg_wrapf(float d) {
  while (d < 0.0f) d += 360.0f;
  while (d >= 360.0f) d -= 360.0f;
  return d;
}

static float cw_distf(float a, float b) { return deg_wrapf(b - a); }

static float ang_difff(float a, float b) {
  float d = a - b;
  if (d < 0.0f) d = -d;
  if (d > 180.0f) d = 360.0f - d;
  return d;
}

static size_t align4(size_t n) { return (n + 3u) & ~(size_t)3u; }

uint32_t gauge_version(void) { return GAUGE_VERSION; }

size_t gauge_scratch_size(int w, int h) {
  if (w < GAUGE_MIN_W || h < GAUGE_MIN_H) {
    w = GAUGE_MIN_W;
    h = GAUGE_MIN_H;
  }
  int w2 = w / 2, h2 = h / 2;
  size_t n2 = (size_t)w2 * (size_t)h2;
  int dmin2 = w2 < h2 ? w2 : h2;
  int maxR = (int)(dmin2 * 0.75f);
  if (maxR < 8) maxR = 8;
  size_t need = 0;
  need += align4(n2 * sizeof(uint8_t));    /* g */
  need += align4(n2 * sizeof(int));        /* mag */
  need += align4(n2 * sizeof(int));        /* gx */
  need += align4(n2 * sizeof(int));        /* gy */
  need += align4(n2 * sizeof(int));        /* acc */
  need += align4((size_t)(maxR + 2) * sizeof(int)); /* hist */
  need += align4(360u * sizeof(float));    /* sc */
  return need;
}

/* Carve one chunk out of the scratch buffer with 4-byte alignment. */
static void *carve(uint8_t **base, size_t *left, size_t n) {
  uintptr_t addr = (uintptr_t)*base;
  uintptr_t aligned = (addr + 3u) & ~(uintptr_t)3u;
  size_t pad = (size_t)(aligned - addr);
  if (pad + n > *left) return NULL;
  *base = (uint8_t *)aligned + n;
  *left -= pad + n;
  return (void *)aligned;
}

/* ---------- image metrics ---------- */

static float blur_metric_f(const uint8_t *px, int w, int h) {
  double m = 0.0, m2 = 0.0;
  long n = 0;
  for (int y = 1; y < h - 1; y += 2) {
    for (int x = 1; x < w - 1; x += 2) {
      int c = px[(size_t)y * w + x];
      int l = c * 4 - px[(size_t)y * w + x - 1] - px[(size_t)y * w + x + 1]
              - px[(size_t)(y - 1) * w + x] - px[(size_t)(y + 1) * w + x];
      m += l;
      m2 += (double)l * l;
      n++;
    }
  }
  if (!n) return 0.0f;
  double var = m2 / n - (m / n) * (m / n);
  return (float)var;
}

/* Returns 0 on success. cx/cy/R in full-res coords.
 * Buffers g/mag/gx/gy/acc/hist are caller scratch (see gauge_scratch_size).
 * track=1: local search in annulus band around (gcx,gcy,gR) full-res. */
static int find_dial_f(const uint8_t *full, int fw, int fh,
                       float *cx, float *cy, float *R, float *support,
                       int track, float gcx, float gcy, float gR,
                       uint8_t *g, int *mag, int *gx, int *gy,
                       int *acc, int *hist, int hist_len) {
  int w = fw / 2, h = fh / 2;
  if (w < 80 || h < 60) return -1;
  size_t n2 = (size_t)w * h;
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int s = full[(size_t)(y * 2) * fw + x * 2] + full[(size_t)(y * 2) * fw + x * 2 + 1]
            + full[(size_t)(y * 2 + 1) * fw + x * 2] + full[(size_t)(y * 2 + 1) * fw + x * 2 + 1];
      g[(size_t)y * w + x] = (uint8_t)(s >> 2);
    }
  }
  double sum = 0.0;
  long ne = 0;
  float bx = gcx * 0.5f, by = gcy * 0.5f, bR = gR * 0.5f;
  for (int y = 1; y < h - 1; y++) {
    for (int x = 1; x < w - 1; x++) {
      int X = g[(size_t)y * w + x + 1] - g[(size_t)y * w + x - 1];
      int Y = g[(size_t)(y + 1) * w + x] - g[(size_t)(y - 1) * w + x];
      gx[(size_t)y * w + x] = X;
      gy[(size_t)y * w + x] = Y;
      int m = (X < 0 ? -X : X) + (Y < 0 ? -Y : Y);
      mag[(size_t)y * w + x] = m;
      if (track) {
        float dx = (float)x - bx, dy = (float)y - by;
        float d = sqrtf(dx * dx + dy * dy);
        float band = bR * 0.55f;
        float dd = d - bR;
        if (dd < 0.0f) dd = -dd;
        if (dd > band) { mag[(size_t)y * w + x] = -1; continue; }
      }
      sum += m;
      ne++;
    }
  }
  double mmean = ne ? sum / ne : 0.0;
  int thr = (int)(mmean * 2.2);
  if (thr < 25) thr = 25;
  memset(acc, 0, n2 * sizeof(int));
  int dmax = w < h ? w : h;
  int maxR = (int)(dmax * 0.75f);
  for (int y = 2; y < h - 2; y++) {
    for (int x = 2; x < w - 2; x++) {
      int m = mag[(size_t)y * w + x];
      if (m < thr) continue;
      float fx = (float)gx[(size_t)y * w + x], fy = (float)gy[(size_t)y * w + x];
      float n = sqrtf(fx * fx + fy * fy);
      if (n < 1e-6f) continue;
      float nx = fx / n, ny = fy / n;
      for (int s = -1; s <= 1; s += 2) {
        for (int t = 8; t < maxR; t += 2) {
          int vx = (int)(x + nx * t * s), vy = (int)(y + ny * t * s);
          if (vx >= 0 && vx < w && vy >= 0 && vy < h) acc[(size_t)vy * w + vx]++;
        }
      }
    }
    if ((y & 31) == 0) { GAUGE_YIELD(); }
  }
  int best = 0, bi = 0;
  for (size_t i = 0; i < n2; i++) {
    if (acc[i] > best) { best = acc[i]; bi = (int)i; }
  }
  long nvote = 0;
  for (int y = 2; y < h - 2; y++)
    for (int x = 2; x < w - 2; x++)
      if (mag[(size_t)y * w + x] >= thr) nvote++;
  if (support) *support = nvote ? (float)best / (float)nvote : 0.0f;
  int ccx = bi % w, ccy = bi / w;

  if (hist_len < maxR + 1) return -1;
  memset(hist, 0, (size_t)(maxR + 1) * sizeof(int));
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      if (mag[(size_t)y * w + x] < thr) continue;
      float dx = (float)(x - ccx), dy = (float)(y - ccy);
      int r = (int)(sqrtf(dx * dx + dy * dy) + 0.5f);
      if (r >= 5 && r <= maxR) hist[r]++;
    }
  }
  int dmin = w < h ? w : h;
  int rmin = (int)(dmin * 0.20f), rmax = (int)(dmin * 0.62f);
  if (track) {
    rmin = (int)(bR * 0.70f);
    rmax = (int)(bR * 1.30f);
  }
  if (rmin < 8) rmin = 8;
  if (rmax > maxR) rmax = maxR;
  if (rmax >= hist_len) rmax = hist_len - 1;
  int br = rmin, bs = -1;
  for (int r = rmin + 1; r < rmax - 1; r++) {
    int s = hist[r - 1] + hist[r] * 2 + hist[r + 1];
    if (s > bs) { bs = s; br = r; }
  }
  /* Tracked band search has far fewer voting edges; lower the peak bar.
   * Global needs a strong peak to avoid locking onto background clutter. */
  if ((track ? best < 12 : best < 40) || bs <= 0) return -1;
  *cx = ccx * 2.0f;
  *cy = ccy * 2.0f;
  *R = br * 2.0f;
  return 0;
}

static float sample_contrast(const uint8_t *g, int w, int h,
                             float x, float y, float pxn, float pyn, float woff,
                             int *c_out) {
  int xi = (int)(x + 0.5f), yi = (int)(y + 0.5f);
  if (xi < 1 || yi < 1 || xi >= w - 1 || yi >= h - 1) return -1.0f;
  int c = g[(size_t)yi * w + xi];
  int x1 = (int)(x + pxn * woff + 0.5f), y1 = (int)(y + pyn * woff + 0.5f);
  int x2 = (int)(x - pxn * woff + 0.5f), y2 = (int)(y - pyn * woff + 0.5f);
  if (x1 < 0 || y1 < 0 || x1 >= w || y1 >= h) return -1.0f;
  if (x2 < 0 || y2 < 0 || x2 >= w || y2 >= h) return -1.0f;
  int sx1 = g[(size_t)y1 * w + x1], sx2 = g[(size_t)y2 * w + x2];
  if (c_out) *c_out = c;
  float s = (sx1 + sx2) * 0.5f - c;
  return s < 0.0f ? 0.0f : s;
}

/* Dual-width (thin+thick needles), outer-weighted, outer-reach continuity. */
static float ray_score_f2(const uint8_t *g, int w, int h,
                          float cx, float cy, float R, float th,
                          float *reach_out) {
  float dx = cosf(th), dy = sinf(th);
  float pxn = -dy, pyn = dx;
  float woff_n = 1.5f;
  float woff_w = R / 14.0f;
  if (woff_w < 3.0f) woff_w = 3.0f;
  if (woff_w > 6.0f) woff_w = 6.0f;
  float acc = 0.0f, wsum = 0.0f;
  int outer_hit = 0, outer_n = 0;
  float r0 = R * 0.25f, r1 = R * 0.95f;
  int steps = (int)(r1 - r0);
  if (steps < 24) steps = 24;
  for (int i = 0; i <= steps; i++) {
    float r = r0 + (r1 - r0) * i / steps;
    float x = cx + dx * r, y = cy + dy * r;
    int c = 0;
    float sn = sample_contrast(g, w, h, x, y, pxn, pyn, woff_n, &c);
    if (sn < 0.0f) continue;
    float sw = sample_contrast(g, w, h, x, y, pxn, pyn, woff_w, NULL);
    float s = sn > sw ? sn : sw;
    float wt = 0.5f + 1.2f * (r / R);
    acc += s * wt;
    wsum += wt;
    if (r > R * 0.55f) {
      outer_n++;
      if (s > 3.0f) outer_hit++;
    }
  }
  if (reach_out) *reach_out = outer_n ? (float)outer_hit / outer_n : 0.0f;
  if (wsum <= 0.0f) return 0.0f;
  float mean = acc / wsum;
  float reach = outer_n ? (float)outer_hit / outer_n : 0.0f;
  return mean * (0.4f + 0.6f * reach);
}

static float hub_darkness_f(const uint8_t *g, int w, int h,
                            float cx, float cy, float R, float th) {
  float dx = cosf(th), dy = sinf(th);
  double s = 0.0;
  int n = 0;
  for (float r = R * 0.10f; r <= R * 0.30f; r += 1.0f) {
    int xi = (int)(cx + dx * r + 0.5f), yi = (int)(cy + dy * r + 0.5f);
    if (xi < 0 || yi < 0 || xi >= w || yi >= h) continue;
    s += g[(size_t)yi * w + xi];
    n++;
  }
  return n ? (float)(s / n) : 255.0f;
}

static float ray_score_f(const uint8_t *g, int w, int h,
                         float cx, float cy, float R, float th) {
  return ray_score_f2(g, w, h, cx, cy, R, th, NULL);
}

static float needle_drun_f(const uint8_t *g, int w, int h,
                           float cx, float cy, float R, float th, float thr) {
  float r = R * 0.12f, r1 = R * 0.95f;
  int dark = 0, gap = 0, best = 0;
  while (r <= r1) {
    float x = cx + cosf(th) * r, y = cy + sinf(th) * r;
    int xi = (int)(x + 0.5f), yi = (int)(y + 0.5f);
    if (xi < 0 || yi < 0 || xi >= w || yi >= h) break;
    if (g[(size_t)yi * w + xi] < thr) {
      dark++;
      gap = 0;
      if (dark > best) best = dark;
    } else if (++gap > 2) {
      break;
    }
    r += 1.0f;
  }
  return best / (R > 1.0f ? R : 1.0f);
}

static float needle_width_f(const uint8_t *g, int w, int h,
                            float cx, float cy, float R, float th,
                            float dial_mean) {
  float dx = cosf(th), dy = sinf(th);
  float pxn = -dy, pyn = dx;
  float thr = dial_mean - 25.0f;
  if (thr < 90.0f) thr = 90.0f;
  if (thr > 180.0f) thr = 180.0f;
  float total = 0.0f;
  for (int k = 0; k < 2; k++) {
    float r = (k == 0 ? 0.40f : 0.60f) * R;
    float x = cx + dx * r, y = cy + dy * r;
    int cnt = 0, gap = 0;
    for (int t = -9; t <= 9; t++) {
      int xx = (int)(x + pxn * t + 0.5f), yy = (int)(y + pyn * t + 0.5f);
      if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
      if (g[(size_t)yy * w + xx] < thr) {
        cnt++;
        gap = 0;
      } else if (++gap >= 2 && t > 2) {
        break;
      }
    }
    total += cnt;
  }
  return total;
}

/* ---------- public API ---------- */

float gauge_value_of(float angle_deg, const gauge_calib_t *calib) {
  float zero_ang = calib ? calib->zero_ang : 0.0f;
  float max_ang = calib ? calib->max_ang : 270.0f;
  float vmin = calib ? calib->vmin : 0.0f;
  float vmax = calib ? calib->vmax : 1.0f;
  float sweep = cw_distf(zero_ang, max_ang);
  if (sweep < 1e-6f) sweep = 270.0f;
  float raw = cw_distf(zero_ang, angle_deg), frac;
  if (raw <= sweep) {
    frac = raw / sweep;
  } else {
    frac = ((360.0f - raw) < (raw - sweep)) ? 0.0f : 1.0f;
  }
  return vmin + frac * (vmax - vmin);
}

gauge_status_t gauge_read_gray(const uint8_t *gray, int w, int h,
                               const gauge_calib_t *calib,
                               const gauge_hint_t *hint,
                               void *scratch, size_t scratch_len,
                               gauge_result_t *out) {
  if (!gray || !calib || !out || !scratch) return GAUGE_ERR_ARG;
  if (w < GAUGE_MIN_W || h < GAUGE_MIN_H) return GAUGE_ERR_SIZE;
  if (gauge_scratch_size(w, h) > scratch_len) return GAUGE_ERR_NOMEM;

  int w2 = w / 2, h2 = h / 2;
  size_t n2 = (size_t)w2 * (size_t)h2;
  int dmin2 = w2 < h2 ? w2 : h2;
  int maxR = (int)(dmin2 * 0.75f);
  if (maxR < 8) maxR = 8;

  /* Partition scratch (4-byte aligned). Order matches gauge_scratch_size. */
  uint8_t *base = (uint8_t *)scratch;
  size_t left = scratch_len;
  uint8_t *g = (uint8_t *)carve(&base, &left, align4(n2 * sizeof(uint8_t)));
  int *mag = (int *)carve(&base, &left, align4(n2 * sizeof(int)));
  int *gx = (int *)carve(&base, &left, align4(n2 * sizeof(int)));
  int *gy = (int *)carve(&base, &left, align4(n2 * sizeof(int)));
  int *acc = (int *)carve(&base, &left, align4(n2 * sizeof(int)));
  int *hist = (int *)carve(&base, &left, align4((size_t)(maxR + 2) * sizeof(int)));
  float *sc = (float *)carve(&base, &left, align4(360u * sizeof(float)));
  if (!g || !mag || !gx || !gy || !acc || !hist || !sc) return GAUGE_ERR_NOMEM;

  int flip = calib->flip ? 1 : 0;

  float cx = 0.0f, cy = 0.0f, R = 0.0f, sup = 0.0f;
  int want_track = (hint && hint->enable) ? 1 : 0;
  int used_track = want_track ? 1 : 0;
  int rc;
  if (want_track) {
    rc = find_dial_f(gray, w, h, &cx, &cy, &R, &sup, 1,
                     hint->gcx, hint->gcy, hint->gR,
                     g, mag, gx, gy, acc, hist, maxR + 2);
    if (rc != 0) {
      used_track = 0; /* local lost: fall back to global */
      rc = find_dial_f(gray, w, h, &cx, &cy, &R, &sup, 0,
                       0.0f, 0.0f, 0.0f,
                       g, mag, gx, gy, acc, hist, maxR + 2);
      if (rc != 0) return GAUGE_ERR_NODIAL;
    } else {
      /* Blend toward lock: kills single-frame quantization jitter. */
      cx = hint->gcx * 0.75f + cx * 0.25f;
      cy = hint->gcy * 0.75f + cy * 0.25f;
      R = hint->gR * 0.75f + R * 0.25f;
    }
  } else {
    rc = find_dial_f(gray, w, h, &cx, &cy, &R, &sup, 0,
                     0.0f, 0.0f, 0.0f,
                     g, mag, gx, gy, acc, hist, maxR + 2);
    if (rc != 0) return GAUGE_ERR_NODIAL;
  }
  out->support = sup;
  out->tracked = used_track;
  float maxRf = (w < h ? w : h) * 0.65f;
  if (R > maxRf) R = maxRf;

  float best_a = 0.0f, best_s = -1.0f, sum = 0.0f, sum2 = 0.0f;
  for (int d = 0; d < 360; d++) {
    float th = d * (float)M_PI / 180.0f;
    float s = ray_score_f(gray, w, h, cx, cy, R, th);
    sc[d] = s;
    sum += s;
    sum2 += s * s;
    if (s > best_s) { best_s = s; best_a = (float)d; }
  }
  float opp_a = deg_wrapf(best_a + 180.0f);
  float best_ref = best_a, best_ref_s = best_s, best_reach = 0.0f;
  float opp_ref = opp_a, opp_ref_s = -1.0f, opp_reach = 0.0f;
  for (float d = best_a - 4.0f; d <= best_a + 4.0f; d += 0.5f) {
    float a = deg_wrapf(d);
    float reach = 0.0f;
    float s = ray_score_f2(gray, w, h, cx, cy, R, a * (float)M_PI / 180.0f, &reach);
    if (s > best_ref_s) { best_ref_s = s; best_ref = a; best_reach = reach; }
  }
  for (float d = opp_a - 4.0f; d <= opp_a + 4.0f; d += 0.5f) {
    float a = deg_wrapf(d);
    float reach = 0.0f;
    float s = ray_score_f2(gray, w, h, cx, cy, R, a * (float)M_PI / 180.0f, &reach);
    if (s > opp_ref_s) { opp_ref_s = s; opp_ref = a; opp_reach = reach; }
  }
  float w_best = 0.0f, w_opp = 0.0f, run_best = 0.0f, run_opp = 0.0f;
  float hub_best = 255.0f, hub_opp = 255.0f;
  {
    double ds = 0.0;
    long dn = 0;
    for (int yy = (int)(cy - R * 0.7f); yy <= (int)(cy + R * 0.7f); yy += 2) {
      for (int xx = (int)(cx - R * 0.7f); xx <= (int)(cx + R * 0.7f); xx += 2) {
        if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
        float ddx = (float)xx - cx, ddy = (float)yy - cy;
        if (ddx * ddx + ddy * ddy > R * R * 0.49f) continue;
        ds += gray[(size_t)yy * w + xx];
        dn++;
      }
    }
    float dial_mean = dn ? (float)(ds / dn) : 160.0f;
    w_best = needle_width_f(gray, w, h, cx, cy, R, best_ref * (float)M_PI / 180.0f, dial_mean);
    w_opp = needle_width_f(gray, w, h, cx, cy, R, opp_ref * (float)M_PI / 180.0f, dial_mean);
    float dr_thr = dial_mean - 30.0f;
    if (dr_thr < 90.0f) dr_thr = 90.0f;
    if (dr_thr > 170.0f) dr_thr = 170.0f;
    run_best = needle_drun_f(gray, w, h, cx, cy, R, best_ref * (float)M_PI / 180.0f, dr_thr);
    run_opp = needle_drun_f(gray, w, h, cx, cy, R, opp_ref * (float)M_PI / 180.0f, dr_thr);
    hub_best = hub_darkness_f(gray, w, h, cx, cy, R, best_ref * (float)M_PI / 180.0f);
    hub_opp = hub_darkness_f(gray, w, h, cx, cy, R, opp_ref * (float)M_PI / 180.0f);
  }
  float ref_a = best_ref, ref_s = best_ref_s, ref_reach = best_reach;
  float alt_a = opp_ref, alt_s = opp_ref_s, alt_reach = opp_reach;
  float ref_w = w_best, alt_w = w_opp;
  float ref_run = run_best, alt_run = run_opp;
  float ref_hub = hub_best, alt_hub = hub_opp;
  int flipped = 0;
  if (flip) {
    ref_a = opp_ref; ref_s = opp_ref_s; ref_reach = opp_reach;
    alt_a = best_ref; alt_s = best_ref_s; alt_reach = best_reach;
    ref_w = w_opp; alt_w = w_best;
    ref_run = run_opp; alt_run = run_best;
    ref_hub = hub_opp; alt_hub = hub_best;
    flipped = 1;
  } else if (hub_best + 25.0f < hub_opp && opp_ref_s > best_ref_s * 0.15f) {
    ref_a = opp_ref; ref_s = opp_ref_s; ref_reach = opp_reach;
    alt_a = best_ref; alt_s = best_ref_s; alt_reach = best_reach;
    ref_w = w_opp; alt_w = w_best;
    ref_run = run_opp; alt_run = run_best;
    ref_hub = hub_opp; alt_hub = hub_best;
    flipped = 1;
  } else if (hub_opp + 25.0f < hub_best) {
    /* raw best already the indicator */
  } else if (run_opp > run_best + 0.15f && opp_ref_s > best_ref_s * 0.25f) {
    ref_a = opp_ref; ref_s = opp_ref_s; ref_reach = opp_reach;
    alt_a = best_ref; alt_s = best_ref_s; alt_reach = best_reach;
    ref_w = w_opp; alt_w = w_best;
    ref_run = run_opp; alt_run = run_best;
    ref_hub = hub_opp; alt_hub = hub_best;
    flipped = 1;
  } else if (run_best > run_opp + 0.15f) {
    /* keep */
  } else if (opp_reach > best_reach + 0.30f && opp_ref_s > best_ref_s * 0.60f) {
    ref_a = opp_ref; ref_s = opp_ref_s; ref_reach = opp_reach;
    alt_a = best_ref; alt_s = best_ref_s; alt_reach = best_reach;
    ref_w = w_opp; alt_w = w_best;
    ref_run = run_opp; alt_run = run_best;
    ref_hub = hub_opp; alt_hub = hub_best;
    flipped = 1;
  }
  float mean = sum / 360.0f;
  float var = sum2 / 360.0f - mean * mean;
  float std = var > 0.0f ? sqrtf(var) : 0.0f;
  float conf = std > 1e-9f ? (ref_s - mean) / std : 0.0f;
  float blur = blur_metric_f(gray, w, h);

  gauge_calib_t vc = *calib;
  float value = gauge_value_of(ref_a, &vc);

  out->cx = cx;
  out->cy = cy;
  out->R = R;
  out->angle_deg = ref_a;
  out->alt_angle_deg = alt_a;
  out->value = value;
  out->conf_sigma = conf;
  out->blur = blur;
  out->tip_score = ref_s;
  out->alt_score = alt_s;
  out->tip_reach = ref_reach;
  out->alt_reach = alt_reach;
  out->tip_width = ref_w;
  out->alt_width = alt_w;
  out->tip_run = ref_run;
  out->alt_run = alt_run;
  out->tip_hub = ref_hub;
  out->alt_hub = alt_hub;
  out->flipped = flipped;
  out->forced_flip = flip;
  out->w = w;
  out->h = h;
  return GAUGE_OK;
}

/* ---------- tracker ---------- */

void gauge_tracker_init(gauge_tracker_t *t) {
  if (!t) return;
  memset(t, 0, sizeof(*t));
  t->init = 0;
}

void gauge_tracker_reset(gauge_tracker_t *t) {
  gauge_tracker_init(t);
}

gauge_status_t gauge_tracker_update(gauge_tracker_t *t,
                                    const gauge_result_t *r,
                                    const gauge_calib_t *calib,
                                    int hint_used,
                                    gauge_stab_t *stab) {
  if (!t || !r || !calib || !stab) return GAUGE_ERR_ARG;

  /* Weak-frame unlock accounting (mirrors app logic): only tracked frames
   * count; a global fallback after a cleared lock is legitimate, not weak. */
  if (t->init && hint_used) {
    if (!r->tracked || r->support < GAUGE_TRACK_WEAK_SUPPORT) {
      t->weak_n++;
    } else {
      t->weak_n = 0;
    }
    if (t->weak_n >= GAUGE_TRACK_WEAK_RESET_N) {
      gauge_tracker_reset(t);
    }
  }

  float pick;
  int used_alt = 0;
  if (!t->init || r->conf_sigma < GAUGE_TRACK_CONF_MIN) {
    t->init = 1;
    t->cx = r->cx;
    t->cy = r->cy;
    t->R = r->R;
    t->ang = r->angle_deg;
    t->n = 1;
    pick = r->angle_deg;
  } else {
    float dcx = r->cx - t->cx;
    if (dcx < 0.0f) dcx = -dcx;
    float dcy = r->cy - t->cy;
    if (dcy < 0.0f) dcy = -dcy;
    float dR = r->R - t->R;
    if (dR < 0.0f) dR = -dR;
    dR /= (t->R > 1.0f ? t->R : 1.0f);
    if (dcx > GAUGE_TRACK_RELOCK_DPX || dcy > GAUGE_TRACK_RELOCK_DPY ||
        dR > GAUGE_TRACK_RELOCK_DR) {
      t->cx = r->cx;
      t->cy = r->cy;
      t->R = r->R;
      t->ang = r->angle_deg;
      t->n = 1;
      pick = r->angle_deg;
    } else {
      t->cx = r->cx;
      t->cy = r->cy;
      t->R = r->R;
      t->n++;
      float d_cur = ang_difff(r->angle_deg, t->ang);
      float d_alt = ang_difff(r->alt_angle_deg, t->ang);
      pick = r->angle_deg;
      if (d_alt + GAUGE_TRACK_ALT_BIAS_DEG < d_cur &&
          r->alt_score > r->tip_score * GAUGE_TRACK_ALT_MIN_RATIO) {
        pick = r->alt_angle_deg;
        used_alt = 1;
      } else if (d_cur > 25.0f && r->tip_score < 8.0f &&
                 r->alt_score > r->tip_score) {
        pick = r->alt_angle_deg;
        used_alt = 1;
      }
      float slew = deg_wrapf(pick - t->ang);
      if (slew > 180.0f) slew -= 360.0f;
      if (slew > GAUGE_TRACK_SLEW_DEG) {
        pick = deg_wrapf(t->ang + GAUGE_TRACK_SLEW_DEG);
      } else if (slew < -GAUGE_TRACK_SLEW_DEG) {
        pick = deg_wrapf(t->ang - GAUGE_TRACK_SLEW_DEG);
      }
      t->ang = pick;
    }
  }

  float sraw = gauge_value_of(pick, calib);
  t->vhist[t->vhn % GAUGE_TRACK_MEDIAN_N] = sraw;
  t->vhn++;
  int nn = t->vhn < GAUGE_TRACK_MEDIAN_N ? t->vhn : GAUGE_TRACK_MEDIAN_N;
  float tmp[GAUGE_TRACK_MEDIAN_N];
  for (int i = 0; i < nn; i++) tmp[i] = t->vhist[i];
  for (int i = 0; i < nn - 1; i++)
    for (int j = i + 1; j < nn; j++)
      if (tmp[j] < tmp[i]) {
        float tt = tmp[i];
        tmp[i] = tmp[j];
        tmp[j] = tt;
      }
  stab->stab_angle = pick;
  stab->stab_value = sraw;
  stab->median_value = tmp[nn / 2];
  stab->used_alt = used_alt;
  stab->locked = t->init ? 1 : 0;
  stab->lock_n = t->n;
  return GAUGE_OK;
}
