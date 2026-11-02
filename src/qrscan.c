// qrscan.c - quirc-backed QR decoder shim.
// Public API: qrscan.h. Handles grayscale copy, decode + mirror retry,
// result mapping, and grid fill for qrscan_dump_grid.
// NOTE: include is "quirc/quirc.h" to match the vendored src/quirc/ layout,
// and mirror retry uses a local flip fallback because this quirc vendor
// does not provide quirc_flip().
#include "qrscan.h"
#include "quirc/quirc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ps_malloc (PSRAM) via Arduino/ESP32 heap; forward-declared for C.
// On non-Arduino hosts (host-side unit builds) fall back to plain malloc.
#ifdef ARDUINO
void *ps_malloc(size_t);
#else
static void *ps_malloc(size_t n) { return malloc(n); }
#endif

// NOTE: quirc_code (~4KB) + quirc_data (~9KB) must NOT live on the stack:
// Arduino loopTask stack is 8KB. Static = safe, single-threaded use.
static struct quirc_code q_code;
static struct quirc_data q_data;

// Local horizontal-mirror fallback (no quirc_flip in this vendor lib).
// tmp is ~3.9KB: static, never stack (loopTask is 8KB).
static uint8_t s_flip_tmp[QRSCAN_MAX_DIM * QRSCAN_MAX_DIM / 8 + 1];
static void qrscan_flip_code(struct quirc_code *code) {
    int n = code->size;
    if (n <= 0 || n > QRSCAN_MAX_DIM) return;
    uint8_t *tmp = s_flip_tmp;
    memset(tmp, 0, sizeof(s_flip_tmp));
    for (int y = 0; y < n; y++) for (int x = 0; x < n; x++) {
        int src = y * n + (n - 1 - x);
        if ((code->cell_bitmap[src >> 3] >> (src & 7)) & 1) {
            int dst = y * n + x;
            tmp[dst >> 3] |= (uint8_t)(1u << (dst & 7));
        }
    }
    memcpy(code->cell_bitmap, tmp, (size_t)((n * n + 7) / 8));
}

const char *qrscan_strerror(int code) {
    switch (code) {
        case 0: return "ok";
        case -1: return "no QR found";
        case -2: return "unsupported version/ecc";
        case -3: return "format info unreadable";
        case -4: return "reed-solomon uncorrectable";
        case -5: return "payload decode fail";
        case -6: return "bad image or no memory";
        default: return "unknown";
    }
}

void qrscan_dump_grid(const qr_workspace_t *ws, int dim) {
    if (dim <= 0 || dim > QRSCAN_MAX_DIM) return;
    for (int r = 0; r < dim; r++) {
        for (int c = 0; c < dim; c++)
            printf("%s", ws->bits[r * dim + c] ? "  " : "##");
        printf("\n");
    }
}

void qrscan_dump_image_ascii(const qr_image_t *img, int threshold) {
    int step_y = img->h / 30 + 1, step_x = img->w / 76 + 1;
    if (step_y < 1) step_y = 1; if (step_x < 1) step_x = 1;
    for (int y = 0; y < img->h; y += step_y) {
        for (int x = 0; x < img->w; x += step_x)
            putchar(img->gray[y * img->w + x] < threshold ? '#' : '.');
        putchar('\n');
    }
}

static int qrscan_map_err(quirc_decode_error_t err) {
    if (err == QUIRC_ERROR_FORMAT_ECC) return -3;
    if (err == QUIRC_ERROR_DATA_ECC) return -4;
    return -5;
}

// Tries all grids currently in qr; first decodable wins. coord_scale maps
// grid corners back to full-res px (2 on the half-res fallback). Returns 0
// with out filled, else the mapped last error. best_plen reports the max
// payload_len seen (0 = grids sample garbage, e.g. blur).
static int qrscan_try_grids(struct quirc *qr, qr_workspace_t *ws,
                            qr_result_t *out, qr_candidate_t *cands,
                            int max_cands, int coord_scale, int *best_plen) {
    int n = quirc_count(qr);
    int last_err = -1;
    int nc = (cands && max_cands > 0) ? (n < max_cands ? n : max_cands) : 0;
    if (best_plen) *best_plen = 0;
    for (int i = 0; i < n; i++) {
        struct quirc_code *code = &q_code;
        struct quirc_data *data = &q_data;
        quirc_decode_error_t err;
        quirc_extract(qr, i, code);
        err = quirc_decode(code, data);
        if (err == QUIRC_ERROR_DATA_ECC) {
            qrscan_flip_code(code);      // mirrored (front camera / flip)
            err = quirc_decode(code, data);
        }
        if (data->payload_len > 0 && best_plen &&
            data->payload_len > *best_plen)
            *best_plen = data->payload_len;
        if (i < nc) {
            cands[i].size = code->size;
            for (int k = 0; k < 4; k++) {
                cands[i].corners[k][0] = code->corners[k].x * coord_scale;
                cands[i].corners[k][1] = code->corners[k].y * coord_scale;
            }
            cands[i].payload_len = data->payload_len;
            cands[i].decode_err =
                (err == QUIRC_SUCCESS) ? 0 : qrscan_map_err(err);
        }
        if (err != QUIRC_SUCCESS) {
            int recoverable =
                (err == QUIRC_ERROR_DATA_ECC ||
                 err == QUIRC_ERROR_DATA_UNDERFLOW ||
                 err == QUIRC_ERROR_DATA_OVERFLOW) &&
                data->payload_len > 0;
            if (!recoverable) {
                last_err = qrscan_map_err(err);
                continue;
            }
            // else: fall through with usable payload
        }
        int len = data->payload_len;
        if (len >= (int)sizeof(out->data)) len = (int)sizeof(out->data) - 1;
        memcpy(out->data, data->payload, (size_t)len);
        out->data[len] = 0;
        out->len = len;
        out->version = data->version;
        out->mask = data->mask;
        switch (data->ecc_level) {
            case QUIRC_ECC_LEVEL_L: out->ecc = 'L'; break;
            case QUIRC_ECC_LEVEL_M: out->ecc = 'M'; break;
            case QUIRC_ECC_LEVEL_Q: out->ecc = 'Q'; break;
            case QUIRC_ECC_LEVEL_H: out->ecc = 'H'; break;
            default: out->ecc = '?'; break;
        }
        out->dim = code->size;
        out->finder_score = 100;
        // Fill ws->bits for dump: quirc bit set = black.
        if (code->size > 0 && code->size <= QRSCAN_MAX_DIM) {
            for (int y = 0; y < code->size; y++) for (int x = 0; x < code->size; x++) {
                int idx = y * code->size + x;
                int black = (code->cell_bitmap[idx >> 3] >> (idx & 7)) & 1;
                ws->bits[y * code->size + x] = black ? 0 : 1;
            }
        }
        if (i < nc) cands[i].decode_err = 0; // accepted (ok/recoverable)
        return 0;
    }
    return last_err;
}

int qrscan_decode_verbose(const qr_image_t *img, qr_workspace_t *ws,
                           qr_result_t *out, int *count_out,
                           qr_candidate_t *cands, int max_cands) {
    if (count_out) *count_out = 0;
    if (cands && max_cands > 0)
        memset(cands, 0, sizeof(*cands) * (size_t)max_cands);
    if (!img || !ws || !out || !img->gray || img->w < 21 || img->h < 21) return -6;
    memset(out, 0, sizeof *out);

    struct quirc *qr = (struct quirc *)ws->quirc;
    if (!qr) {
        qr = quirc_new();
        if (!qr) return -6;
        ws->quirc = qr; ws->qw = 0; ws->qh = 0;
    }
    if (ws->qw != img->w || ws->qh != img->h) {
        if (quirc_resize(qr, img->w, img->h) < 0) return -6;
        ws->qw = img->w; ws->qh = img->h;
    }
    int qw = 0, qh = 0;
    uint8_t *buf = quirc_begin(qr, &qw, &qh);
    if (!buf) return -6;
    // Auto-levels: OV2640 grayscale often sits in a narrow washed band
    // (glare/auto-exposure), which starves quirc's adaptive threshold.
    // Stretch sampled min/max to full range; identity when already full.
    {
        size_t npix = (size_t)img->w * (size_t)img->h;
        int lo = 255, hi = 0;
        for (size_t i = 0; i < npix; i += 4) {
            int v = img->gray[i];
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        if (hi - lo >= 40 && lo >= 0 && hi <= 255 && hi > lo) {
            int range = hi - lo;
            for (size_t i = 0; i < npix; i++) {
                int v = ((int)img->gray[i] - lo) * 255 / range;
                buf[i] = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
            }
        } else {
            memcpy(buf, img->gray, npix);
        }
    }
    quirc_end(qr);

    int n = quirc_count(qr);
    int best_plen = 0;
    int rc = qrscan_try_grids(qr, ws, out, cands, max_cands, 1, &best_plen);
    int full_n = n;
    if (count_out) *count_out = n;
    if (rc == 0)
        return 0;
    // Second-chance pass: 2x-downsampled copy (320x240). Box averaging
    // smooths single-pixel noise/blur that kills full-res sampling.
    // PSRAM scratch, allocated once. Corners scaled back x2 for logs.
    static uint8_t *s_half = NULL;
    static int s_half_tried = 0;
    if (img->w == 640 && img->h == 480) {
        if (!s_half && !s_half_tried) {
            s_half = (uint8_t *)ps_malloc(320 * 240);
            s_half_tried = 1;
        }
        if (s_half) {
            for (int y = 0; y < 240; y++)
                for (int x = 0; x < 320; x++) {
                    size_t i0 = ((size_t)(y * 2) * 640) + (x * 2);
                    s_half[(size_t)y * 320 + x] = (uint8_t)(
                        ((int)img->gray[i0] + img->gray[i0 + 1] +
                         img->gray[i0 + 640] + img->gray[i0 + 641]) >> 2);
                }
            if (quirc_resize(qr, 320, 240) == 0) {
                ws->qw = 320;
                ws->qh = 240;
                uint8_t *hb = quirc_begin(qr, NULL, NULL);
                if (hb) {
                    memcpy(hb, s_half, 320 * 240);
                    quirc_end(qr);
                    n = quirc_count(qr);
                    int half_best = 0;
                    // Stash half-res candidates after full-res ones.
                    qr_candidate_t *hc =
                        (cands && full_n < max_cands) ? &cands[full_n] : NULL;
                    int hmax =
                        (cands && full_n < max_cands) ? max_cands - full_n : 0;
                    rc = qrscan_try_grids(qr, ws, out, hc, hmax, 2,
                                          &half_best);
                    if (count_out)
                        *count_out =
                            full_n * 100 + n; // full*100+half grid counts
                    if (rc == 0)
                        return 0;
                    if (half_best > best_plen)
                        best_plen = half_best;
                }
            }
            // Restore full-res sizing for the next frame.
            quirc_resize(qr, img->w, img->h);
            ws->qw = img->w;
            ws->qh = img->h;
        }
    }
    // Last resort: 1px-shift jiggle. Re-detection in a shifted image makes
    // different rounding/threshold choices; if sampling is knife-edge, one
    // of the 4 phases often locks. Edge-replicated so no resize is needed.
    // PSRAM scratch, allocated once. Only when grids exist but fail: with
    // no grid at all, extra passes just burn frame rate.
    static uint8_t *s_shift = NULL;
    static int s_shift_tried = 0;
    if (best_plen == 0 && full_n > 0 && img->w >= 64 && img->h >= 64) {
        size_t npix = (size_t)img->w * (size_t)img->h;
        if (!s_shift && !s_shift_tried) {
            s_shift = (uint8_t *)ps_malloc(npix);
            s_shift_tried = 1;
        }
        if (s_shift) {
            static const int8_t kdx[3] = {1, 0, 1};
            static const int8_t kdy[3] = {0, 1, 1};
            for (int v = 0; v < 3; v++) {
                int dx = kdx[v], dy = kdy[v];
                for (int y = 0; y < img->h; y++) {
                    int sy = y + dy;
                    if (sy >= img->h) sy = img->h - 1;
                    size_t dr = (size_t)y * img->w;
                    size_t sr = (size_t)sy * img->w;
                    for (int x = 0; x < img->w; x++) {
                        int sx = x + dx;
                        if (sx >= img->w) sx = img->w - 1;
                        s_shift[dr + x] = img->gray[sr + sx];
                    }
                }
                if ((ws->qw != img->w || ws->qh != img->h) &&
                    quirc_resize(qr, img->w, img->h) != 0)
                    break;
                ws->qw = img->w;
                ws->qh = img->h;
                uint8_t *jb = quirc_begin(qr, NULL, NULL);
                if (!jb)
                    break;
                memcpy(jb, s_shift, npix);
                quirc_end(qr);
                n = quirc_count(qr);
                int jbest = 0;
                qr_candidate_t *jc =
                    (cands && full_n < max_cands) ? &cands[full_n] : NULL;
                int jmax =
                    (cands && full_n < max_cands) ? max_cands - full_n : 0;
                int jrc = qrscan_try_grids(qr, ws, out, jc, jmax, 1, &jbest);
                if (count_out)
                    *count_out = full_n * 100 + n;
                if (jrc == 0)
                    return 0;
                if (jbest > best_plen)
                    best_plen = jbest;
            }
            quirc_resize(qr, img->w, img->h);
            ws->qw = img->w;
            ws->qh = img->h;
        }
    }
    (void)best_plen;
    return rc;
}

int qrscan_decode(const qr_image_t *img, qr_workspace_t *ws, qr_result_t *out) {
    return qrscan_decode_verbose(img, ws, out, NULL, NULL, 0);
}

void qrscan_free(qr_workspace_t *ws) {
    if (!ws) return;
    if (ws->quirc) {
        quirc_destroy((struct quirc *)ws->quirc);
        ws->quirc = NULL;
    }
    ws->qw = 0;
    ws->qh = 0;
}
