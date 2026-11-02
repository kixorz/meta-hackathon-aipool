// qrscan.h - QR decoder for the ESP32-CAM setup flow.
// Backend: quirc (vendored in src/quirc/, MIT). Public API is a thin wrapper
// so callers don't change if the backend changes.
// Supports versions 1-40, all ECC levels, tilted/rotated frames.
#ifndef QRSCAN_H
#define QRSCAN_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QRSCAN_MAX_DATA 256
#define QRSCAN_MAX_DIM  177  // version 40 = 177x177
#define QRSCAN_MAX_CW   4000 // kept for ABI compat, unused by quirc backend

typedef struct {
    const uint8_t *gray;  // 8-bit grayscale, row-major
    int w, h;             // image dimensions
} qr_image_t;

typedef struct {
    char data[QRSCAN_MAX_DATA]; // NUL-terminated decoded bytes (truncated to 255)
    int len;                    // strlen without NUL
    int version;                // 1..40
    char ecc;                   // 'L','M','Q','H'
    int mask;                   // 0..7
    int finder_score;           // 0..100 (100 = decoded OK)
    int dim;                    // modules per side
} qr_result_t;

// Caller-owned workspace. Holds quirc handle + last grid for dump.
// quirc pixel buffer is heap-allocated once per size (via quirc_resize),
// not per frame.
// WARNING: sizeof(qr_workspace_t) is ~39KB (177x177 grid + compat buffers).
// NEVER allocate on the stack (loopTask is 8KB) — use static storage or
// heap (ps_malloc). Stack allocation corrupts memory and trips TG1WDT.
typedef struct {
    void *quirc;                       // struct quirc* (opaque here)
    int qw, qh;                        // current quirc buffer size
    uint8_t bits[QRSCAN_MAX_DIM * QRSCAN_MAX_DIM]; // last grid, 1=white,0=black
    uint8_t cw[QRSCAN_MAX_CW];         // unused (ABI compat)
    uint8_t data_cw[QRSCAN_MAX_CW];    // unused (ABI compat)
    int cw_total;
    int data_total;
} qr_workspace_t;

// Returns 0 on success (result filled), <0 on failure.
// err codes: -1 no QR found, -2 unsupported, -3 format info bad,
//            -4 ecc uncorrectable, -5 payload decode fail, -6 bad image/nomem.
//
// Recoverable quirc errors (DATA_ECC after mirror retry, DATA_UNDERFLOW,
// DATA_OVERFLOW) still return 0 when data->payload_len > 0, because a short
// WIFI:... payload (~30 chars) is usually intact. Callers that need strict
// ECC should inspect out->ecc/version instead.
// Workspace lifecycle: allocate static or heap (NEVER stack), zero-init,
// reuse across frames; call qrscan_free(&ws) when done to release the
// quirc handle. qrscan_decode lazily allocates the handle on first use.
int qrscan_decode(const qr_image_t *img, qr_workspace_t *ws, qr_result_t *out);

// Release the quirc handle held by ws (safe on zero-init / double-free).
// Keeps the setup flow's retry-forever loop from leaking PSRAM.
void qrscan_free(qr_workspace_t *ws);

const char *qrscan_strerror(int code);

// Per-candidate diagnostics for the setup loop (no quirc types leak).
#define QRSCAN_MAX_CANDS 8
typedef struct {
  int size;              // grid dim (modules per side), -1 if extract failed
  int corners[4][2];     // image px, TL clockwise
  int decode_err;        // 0=ok, else qrscan -code (-3/-4/-5)
  int payload_len;       // data->payload_len even on failure (0 if none)
} qr_candidate_t;

// Verbose decode: like qrscan_decode but also reports candidate count and
// per-candidate geometry/errors. count_out=quirc_count, cands filled up to
// max_cands. Return/out semantics identical to qrscan_decode.
int qrscan_decode_verbose(const qr_image_t *img, qr_workspace_t *ws,
                          qr_result_t *out, int *count_out,
                          qr_candidate_t *cands, int max_cands);

// --- observability helpers (host + device safe) ---
void qrscan_dump_grid(const qr_workspace_t *ws, int dim);
void qrscan_dump_image_ascii(const qr_image_t *img, int threshold);

#ifdef __cplusplus
}
#endif
#endif
