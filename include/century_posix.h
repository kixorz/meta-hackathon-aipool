#pragma once

#include "century.h"

#ifdef __cplusplus
extern "C" {
#endif

/* POSIX serial backend (Linux/macOS). Excluded on bare metal.
 * Depends on termios/poll only. */

typedef struct century_posix century_posix_t;

century_posix_t *century_posix_open(const char *device, uint32_t baud);
void century_posix_close(century_posix_t *p);

/* Set true to drive RTS as DE/RE (for modules without auto-direction).
 * Default false. */
void century_posix_set_rts_control(century_posix_t *p, bool enable);

/* Fill a transport backed by this handle (valid until close). */
century_transport_t century_posix_transport(century_posix_t *p);

#ifdef __cplusplus
}
#endif
