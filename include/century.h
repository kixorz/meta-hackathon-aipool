#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "century_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

#define CENTURY_DEFAULT_ADDR 0x15u
#define CENTURY_ADDR_MIN 1u
#define CENTURY_ADDR_MAX 247u

#define CENTURY_MIN_RPM 600u
#define CENTURY_MAX_RPM 3450u
#define CENTURY_RPM_STEP 50u

/* Request tag byte after FUNC (ESPHome ack_=0x20). */
#define CENTURY_REQ_TAG 0x20u
/* Response ACK byte. */
#define CENTURY_ACK 0x10u

/* Function codes (custom, NOT standard Modbus). */
#define CENTURY_FUNC_RUN 0x41u
#define CENTURY_FUNC_STOP 0x42u
#define CENTURY_FUNC_STATUS 0x43u
#define CENTURY_FUNC_SET_DEMAND 0x44u
#define CENTURY_FUNC_READ_SENSOR 0x45u
#define CENTURY_FUNC_CONFIG_RW 0x64u
#define CENTURY_FUNC_STORE_CONFIG 0x65u

/* Known sensor pages/addresses (from EPC protocol + reference impls).
 * Generic century_read_sensor() covers everything; these are shortcuts. */
#define CENTURY_SENS_RPM_PAGE 0u
#define CENTURY_SENS_RPM_ADDR 0u       /* verified live on EVO */
#define CENTURY_SENS_RPM2_PAGE 3u
#define CENTURY_SENS_RPM2_ADDR 4u      /* alt RPM page (classic VGreen) */
#define CENTURY_SENS_DEMAND_PAGE 0u
#define CENTURY_SENS_DEMAND_ADDR 3u
#define CENTURY_SENS_WATTS_PAGE 0u
#define CENTURY_SENS_WATTS_ADDR 5u      /* live power, watts, scale 1 */
#define CENTURY_SENS_TEMP_PAGE 0u
#define CENTURY_SENS_TEMP_ADDR 7u      /* ambient, signed raw / 128 = degC */
#define CENTURY_SENS_PREVFAULT_PAGE 0u
#define CENTURY_SENS_PREVFAULT_ADDR 9u  /* low byte is fault code */
#define CENTURY_SENS_FAULT_PAGE 1u
#define CENTURY_SENS_FAULT1_ADDR 0x15u  /* fault slots 1..4 at 0x15..0x18 */
#define CENTURY_SENS_FAULT2_ADDR 0x16u
#define CENTURY_SENS_FAULT3_ADDR 0x17u
#define CENTURY_SENS_FAULT4_ADDR 0x18u

/* Config-write page/address for the auxiliary relay (0x64). */
#define CENTURY_CFG_PAGE1_WRITE 0x81u
#define CENTURY_CFG_AUX_RELAY_ADDR 0x07u
#define CENTURY_AUX_ON_VALUE 0x00u
#define CENTURY_AUX_OFF_VALUE 0x01u

/* Ambient-temp conversion: raw is signed, 1 LSB = 1/128 degC. */
#define CENTURY_TEMP_MILLIC(raw) ((int32_t)(int16_t)(raw) * 1000 / 128)

/* Demand modes for 0x44 payload byte 0. */
typedef enum {
    CENTURY_MODE_SPEED = 0,
    CENTURY_MODE_TORQUE = 1
} century_mode_t;

typedef enum {
    CENTURY_STATUS_STOPPED = 0x00,
    CENTURY_STATUS_BOOTING = 0x09,
    CENTURY_STATUS_ACCELERATING = 0x0A,
    CENTURY_STATUS_RUNNING = 0x0B,
    CENTURY_STATUS_FAULT = 0x20,
    CENTURY_STATUS_UNKNOWN = 0xFF
} century_status_t;

const char *century_status_str(century_status_t s);

/* ------------------------------------------------------------------ */
/* Errors                                                              */
/* ------------------------------------------------------------------ */

typedef enum {
    CENTURY_OK = 0,
    CENTURY_ERR_INVALID_ARG = -1,
    CENTURY_ERR_BUFFER_TOO_SMALL = -2,
    CENTURY_ERR_CRC = -3,
    CENTURY_ERR_SHORT = -4,
    CENTURY_ERR_ADDR_MISMATCH = -5,
    CENTURY_ERR_FUNC_MISMATCH = -6,
    CENTURY_ERR_NACK = -7,
    CENTURY_ERR_TIMEOUT = -8,
    CENTURY_ERR_IO = -9,
    CENTURY_ERR_REPLY_MISMATCH = -10 /* valid frame but content echo wrong */
} century_err_t;

const char *century_strerror(century_err_t err);

/* ------------------------------------------------------------------ */
/* Logging (callback, no printf in core)                               */
/* ------------------------------------------------------------------ */

typedef enum {
    CENTURY_LOG_DEBUG = 0,
    CENTURY_LOG_INFO = 1,
    CENTURY_LOG_WARN = 2,
    CENTURY_LOG_ERROR = 3
} century_log_level_t;

/* Preformatted message, no varargs (bare-metal friendly). */
typedef void (*century_log_fn_t)(void *log_ctx, century_log_level_t level,
                                 const char *file, int line, const char *msg);

/* ------------------------------------------------------------------ */
/* Transport abstraction                                               */
/* ------------------------------------------------------------------ */

/*
 * read(): blocking read up to max_len bytes, waiting at most timeout_ms.
 *   return >0 : bytes read (may be a partial frame; client accumulates)
 *   return 0  : timeout, no data
 *   return <0 : I/O error
 * write(): return 0 on success, <0 on error. Must write all len bytes.
 * delay_ms(): may be NULL (no gap delay then).
 * millis(): may be NULL (client does a single read instead of deadline loop).
 * set_tx_enable(): may be NULL (auto-direction RS485 or always-on).
 */
typedef struct {
    void *ctx;
    int (*write)(void *ctx, const uint8_t *buf, size_t len);
    int (*read)(void *ctx, uint8_t *buf, size_t max_len, uint32_t timeout_ms);
    void (*delay_ms)(uint32_t ms);
    uint32_t (*millis)(void);
    void (*set_tx_enable)(void *ctx, bool tx);
} century_transport_t;

typedef struct {
    uint8_t addr;
    century_transport_t *tr;
    uint32_t timeout_ms;   /* per-attempt response timeout (150..400) */
    uint32_t gap_ms;       /* inter-frame gap (~10) */
    int retries;           /* total attempts, up to 5 */
    bool allow_quirk;      /* accept EVO frames missing FUNC echo */
    century_log_fn_t log_fn;
    void *log_ctx;
    uint8_t last_nack;       /* NACK/exception code from most recent request (0 = none) */
} century_client_t;

void century_client_init(century_client_t *c, uint8_t addr, century_transport_t *tr);
void century_client_set_log(century_client_t *c, century_log_fn_t fn, void *ctx);

/* ------------------------------------------------------------------ */
/* Codec (no I/O)                                                      */
/* ------------------------------------------------------------------ */

uint16_t century_crc16(const uint8_t *d, size_t len);

/*
 * Build TX frame [ADDR][FUNC][0x20][PL...][CRClo][CRChi] into out.
 * Returns total length (>=5) on success, negative century_err_t on failure.
 */
int century_build_frame(uint8_t addr, uint8_t func,
                        const uint8_t *payload, size_t payload_len,
                        uint8_t *out, size_t out_cap);

typedef struct {
    bool quirk;            /* true if FUNC echo was missing (EVO quirk) */
    const uint8_t *data;   /* points into rx (after ACK stripped) */
    size_t data_len;
} century_parsed_t;

/*
 * Validate RX frame against expected addr/func.
 * Accepts both normal [ADDR][FUNC][0x10][DATA][CRC]
 * and quirk   [ADDR][0x10][DATA][CRC] (if allow_quirk).
 * Returns CENTURY_OK and fills *info (data points into rx).
 */
century_err_t century_parse_frame(const uint8_t *rx, size_t rx_len,
                                  uint8_t exp_addr, uint8_t exp_func,
                                  bool allow_quirk, century_parsed_t *info);

/* ------------------------------------------------------------------ */
/* High-level client (blocking, retries)                               */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t page_echo;
    uint8_t addr_echo;
    uint16_t raw;        /* unscaled value */
    uint16_t scaled;     /* raw / scale */
    bool unsupported;    /* true if device returned 0xFFFF */
} century_sensor_t;

century_err_t century_get_status(century_client_t *c, century_status_t *out);
century_err_t century_run(century_client_t *c);
century_err_t century_stop(century_client_t *c);

/* Warn-and-round: clamps to 600..3450, rounds to 50-step, logs WARN on change. */
century_err_t century_set_demand(century_client_t *c, century_mode_t mode, uint16_t rpm);

/* Safe sequence: set demand first, then run (motor drops to stop otherwise). */
century_err_t century_set_demand_and_run(century_client_t *c, uint16_t rpm);

/* Fully generic sensor read. scale==0 means no scaling (scaled=raw). */
century_err_t century_read_sensor(century_client_t *c, uint8_t page, uint8_t addr,
                                  uint16_t scale, century_sensor_t *out);

/* Convenience wrappers (generic underneath). */
century_err_t century_read_rpm(century_client_t *c, uint16_t *rpm_out);
century_err_t century_read_demand_rpm(century_client_t *c, uint16_t *rpm_out);
century_err_t century_read_power_watts(century_client_t *c, uint16_t *watts_out);
century_err_t century_read_ambient_temp_raw(century_client_t *c, int16_t *raw_out);
century_err_t century_read_fault_code(century_client_t *c, uint8_t page, uint8_t addr,
                                      uint8_t *code_out);

/* Fault-code text (EPC table). */
const char *century_fault_str(uint8_t code);

/* Aux relay + config store (0x64 / 0x65). */
century_err_t century_set_aux_relay(century_client_t *c, bool on);
century_err_t century_store_config(century_client_t *c);

/* NACK/exception support: motor replies [ADDR][FUNC|0x80][CODE][CRC].
 * century_last_nack() returns the code from the most recent request (0 = none). */
uint8_t century_last_nack(const century_client_t *c);
const char *century_nack_str(uint8_t code);

/* ------------------------------------------------------------------ */
/* Address scan / auto-detect                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t addr;
    uint8_t status_byte; /* raw 0x43 DATA[0] */
} century_scan_hit_t;

/* STATUS sweep over [start, end] (clamped to 1..247). Stores up to cap hits;
 * *found is the true total even if it exceeds cap. Quiet: per-address misses
 * never log (a full sweep would otherwise spam 200+ WARNs). */
century_err_t century_scan(century_client_t *c, uint8_t start, uint8_t end,
                           uint32_t per_addr_timeout_ms,
                           century_scan_hit_t *out, size_t cap, size_t *found);

century_err_t century_auto_detect(century_client_t *c, uint8_t *addr_out);

#ifdef __cplusplus
}
#endif
