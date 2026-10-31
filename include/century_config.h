#pragma once

/* Tunables for libcentury. Override via -DCENTURY_xxx on the compiler line
 * if you need smaller RAM on bare metal. */

#ifndef CENTURY_MAX_FRAME
#define CENTURY_MAX_FRAME 64
#endif

#ifndef CENTURY_MAX_PAYLOAD
#define CENTURY_MAX_PAYLOAD 32
#endif

/* Max formatted log message (stack buffer). */
#ifndef CENTURY_LOG_MSG_MAX
#define CENTURY_LOG_MSG_MAX 128
#endif

/* Set to 1 to use a 256-entry CRC table (faster, +512B flash). */
#ifndef CENTURY_CRC_TABLE
#define CENTURY_CRC_TABLE 0
#endif
