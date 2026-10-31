/* gauge.h — portable analog-gauge reader core.
 *
 * Pure C99, no file I/O, no malloc, no platform headers.
 * Input: 8-bit grayscale image. Output: dial geometry + needle angle + value.
 *
 * Angle convention (image coords, x right, y DOWN):
 *   degrees, 0 = east, 90 = south, 180 = west, 270 = north, clockwise positive.
 * Value maps clockwise from zero_ang through max_ang (sweep up to 360 deg).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GAUGE_VERSION_MAJOR 0
#define GAUGE_VERSION_MINOR 1
#define GAUGE_VERSION_PATCH 0

/* Packed 0xMMmmpp version, e.g. 0x000100 for 0.1.0. */
#define GAUGE_VERSION ((uint32_t)((GAUGE_VERSION_MAJOR << 16) | (GAUGE_VERSION_MINOR << 8) | GAUGE_VERSION_PATCH))

/* Minimum input sanity limits (matches on-device guard). */
#define GAUGE_MIN_W 160
#define GAUGE_MIN_H 120

/* Tracker tuning (ported from app_httpd stabilization). Exposed so host
 * tests and alternative deployments can reason about them. */
#define GAUGE_TRACK_RELOCK_DPX   25.0f   /* dial jump => scene change, re-lock */
#define GAUGE_TRACK_RELOCK_DPY   25.0f
#define GAUGE_TRACK_RELOCK_DR    0.35f    /* relative radius jump => re-lock */
#define GAUGE_TRACK_ALT_BIAS_DEG 12.0f    /* alt wins if this much closer to lock */
#define GAUGE_TRACK_ALT_MIN_RATIO 0.35f   /* alt_score > tip_score * this */
#define GAUGE_TRACK_SLEW_DEG     15.0f    /* max angle step per tracked reading */
#define GAUGE_TRACK_WEAK_SUPPORT 0.010f   /* support below => weak frame */
#define GAUGE_TRACK_WEAK_RESET_N 3        /* consecutive weak frames => unlock */
#define GAUGE_TRACK_MEDIAN_N     9        /* median window over stabilized values */
#define GAUGE_TRACK_CONF_MIN     2.0f     /* below => (re-)lock without hysteresis */

/** Status codes returned by gauge_read_gray() / gauge_tracker_update(). */
typedef enum {
  GAUGE_OK = 0,       /**< success */
  GAUGE_ERR_ARG = -1, /**< NULL pointer / invalid calibration */
  GAUGE_ERR_SIZE = -2,/**< image smaller than GAUGE_MIN_W x GAUGE_MIN_H */
  GAUGE_ERR_NODIAL = -3, /**< dial circle not found */
  GAUGE_ERR_NOMEM = -4   /**< scratch buffer too small (see gauge_scratch_size) */
} gauge_status_t;

/** Per-frame reading. Field-compatible with the legacy on-device struct. */
typedef struct {
  float cx, cy, R;              /**< dial center + radius, full-res pixels */
  float angle_deg;              /**< picked tip angle, 0=E,90=S y-down, clockwise+ */
  float alt_angle_deg;          /**< opposite-end (tail) candidate angle */
  float value;                  /**< calibrated value for angle_deg */
  float conf_sigma;             /**< (tip-mean)/std over 360 deg scan; <2 = weak */
  float blur;                   /**< Laplacian variance; <20 blurry, <60 soft */
  float tip_score, alt_score;   /**< outer-weighted ray scores */
  float tip_reach, alt_reach;   /**< outer-half continuity 0..1 */
  float tip_width, alt_width;   /**< hub width estimate (px) */
  float tip_run, alt_run;       /**< continuous dark run from hub (units of R) */
  float tip_hub, alt_hub;       /**< mean gray near hub (stub=dark, hand=bright) */
  float support;                /**< dial vote support ratio (circle evidence) */
  int tracked;                  /**< 1 if local tracked search was used */
  int flipped;                  /**< 1 if auto rule flipped vs raw max */
  int forced_flip;              /**< echo of calib.flip */
  int w, h;                     /**< echoed input dimensions */
} gauge_result_t;

/** Fixed gauge calibration: which angles map to which values. */
typedef struct {
  float zero_ang;  /**< angle of vmin (deg, convention above) */
  float max_ang;   /**< angle of vmax (deg) */
  float vmin;      /**< value at zero_ang */
  float vmax;      /**< value at max_ang */
  int flip;        /**< nonzero forces opposite-hand pick */
} gauge_calib_t;

/** Optional dial-position hint for fast local search (camera is fixed). */
typedef struct {
  int enable;           /**< 0 = global search, 1 = local search around guess */
  float gcx, gcy, gR;   /**< guess center + radius, full-res pixels */
} gauge_hint_t;

/** Temporal stabilization state. Allocate statically, init once.
 * Replaces the file-static g_lock + vhist previously in app_httpd.cpp. */
typedef struct {
  int init;                              /**< 0 = unlocked, 1 = locked */
  float cx, cy, R, ang;                  /**< locked geometry + angle */
  int n;                                 /**< readings since lock */
  float vhist[GAUGE_TRACK_MEDIAN_N];     /**< ring buffer of stabilized values */
  int vhn;                               /**< total pushes (for fill count) */
  int weak_n;                            /**< consecutive weak tracked frames */
} gauge_tracker_t;

/** Stabilized output of gauge_tracker_update(). */
typedef struct {
  float stab_angle;    /**< hysteresis + slew-limited angle */
  float stab_value;    /**< value of stab_angle under current calib */
  float median_value;  /**< median of last up-to-9 stab_values */
  int used_alt;        /**< 1 if alt candidate was picked this step */
  int locked;          /**< 1 if tracker holds a lock after this step */
  int lock_n;          /**< readings since lock */
} gauge_stab_t;

/** @return packed version (GAUGE_VERSION). */
uint32_t gauge_version(void);

/**
 * Worst-case scratch bytes needed for gauge_read_gray() at (w,h).
 * Scratch is carved internally for the downsampled vote buffers and the
 * 360-entry angle scan; the caller owns the memory (static array, PSRAM,
 * stack spill, ...). Pass the same pointer+length to gauge_read_gray().
 * Contents are throwaway; may be reused across calls/frames of same size.
 */
size_t gauge_scratch_size(int w, int h);

/**
 * Read one frame. Pure function of (gray, calib, hint); no static state.
 *
 * @param gray   w*h bytes, row-major 8-bit gray. Must outlive the call only.
 * @param w,h    dimensions. w>=160, h>=120 required, else GAUGE_ERR_SIZE.
 * @param calib  zero/max angles + value range + flip. Sweep cw(zero->max);
 *               a sweep <1e-6 deg falls back to 270 deg.
 * @param hint   NULL = global search. hint->enable = local band search with
 *               fallback to global when lost; blends result 0.75*guess+0.25*new
 *               to kill single-frame quantization jitter.
 * @param scratch scratch buffer of at least gauge_scratch_size(w,h) bytes.
 * @param scratch_len bytes available at scratch.
 * @param out    result (also carries value mapped with calib).
 * @return GAUGE_OK on success, GAUGE_ERR_* otherwise. On error *out is
 *         left untouched except w/h echo is NOT guaranteed.
 */
gauge_status_t gauge_read_gray(const uint8_t *gray, int w, int h,
                               const gauge_calib_t *calib,
                               const gauge_hint_t *hint,
                               void *scratch, size_t scratch_len,
                               gauge_result_t *out);

/**
 * Map an angle to a value under calib (clockwise sweep, endpoint clamp).
 * Values outside the sweep clamp to the nearer endpoint (vmin or vmax).
 */
float gauge_value_of(float angle_deg, const gauge_calib_t *calib);

/** Init / reset tracker state (e.g. on `?reset=1` or scene change). */
void gauge_tracker_init(gauge_tracker_t *t);
void gauge_tracker_reset(gauge_tracker_t *t);

/**
 * Fold one raw reading into the tracker: re-lock on scene jumps or weak
 * confidence, hysteresis between tip/alt candidates near the lock, slew
 * limit (needle is slow), plus median-of-9 over stabilized values.
 *
 * @param t         tracker state (updated in place).
 * @param r         raw per-frame result from gauge_read_gray().
 * @param calib     same calib used for the read (for value mapping).
 * @param hint_used nonzero if the read attempted a tracked search
 *                  (r->tracked seeded from a lock). Only tracked frames feed
 *                  the weak-frame unlock counter; a global fallback after a
 *                  cleared lock is legitimate, not weak.
 * @param stab      stabilized output.
 * @return GAUGE_OK, or GAUGE_ERR_ARG on NULL pointers.
 */
gauge_status_t gauge_tracker_update(gauge_tracker_t *t,
                                    const gauge_result_t *r,
                                    const gauge_calib_t *calib,
                                    int hint_used,
                                    gauge_stab_t *stab);

#ifdef __cplusplus
}
#endif
