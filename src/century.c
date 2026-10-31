#include "century.h"

/* ------------------------------------------------------------------ */
/* century.c — codec + synchronous client for Century VGreen / VGreen EVO. */
/*                                                                     */
/* Wire format (see poolpump.md):                                     */
/*   TX: [ADDR][FUNC][0x20][PAYLOAD...][CRClo][CRChi]                   */
/*   RX: [ADDR][FUNC][0x10][DATA...][CRClo][CRChi]                      */
/*   EVO quirk: some fw omits FUNC echo: [ADDR][0x10][DATA][CRC]        */
/*                                                                     */
/* No malloc, no printf in core. Logging goes through client callback. */
/* ------------------------------------------------------------------ */

/* -- internal log helpers (manual formatting, no snprintf) ----------- */

static void log_msg(const century_client_t *c, century_log_level_t lvl,
                    const char *file, int line, const char *msg) {
    if (c && c->log_fn) {
        c->log_fn(c->log_ctx, lvl, file, line, msg);
    }
}

/* Format u32 decimal into buf, returns chars written. */
static size_t fmt_u32(char *buf, size_t cap, uint32_t v) {
    char tmp[11];
    size_t n = 0;
    if (v == 0) {
        if (cap > 0) buf[0] = '0';
        return 1;
    }
    while (v > 0 && n < sizeof(tmp)) {
        tmp[n++] = (char)('0' + (v % 10));
        v /= 10;
    }
    size_t out = 0;
    while (n > 0 && out < cap) {
        buf[out++] = tmp[--n];
    }
    return out;
}

static void log_u32(const century_client_t *c, century_log_level_t lvl,
                    const char *file, int line,
                    const char *prefix, uint32_t v) {
    char buf[CENTURY_LOG_MSG_MAX];
    size_t i = 0;
    while (prefix[i] != '\0' && i < sizeof(buf) - 14) {
        buf[i] = prefix[i];
        i++;
    }
    i += fmt_u32(buf + i, sizeof(buf) - 1 - i, v);
    buf[i] = '\0';
    log_msg(c, lvl, file, line, buf);
}

/* "TAG 15 43 20 ..." hex dump into callback (truncated to fit). */
static void log_hex(const century_client_t *c, century_log_level_t lvl,
                    const char *file, int line,
                    const char *tag, const uint8_t *b, size_t len) {
    static const char HEX[] = "0123456789ABCDEF";
    char buf[CENTURY_LOG_MSG_MAX];
    size_t i = 0;
    while (tag[i] != '\0' && i < sizeof(buf) - 4) {
        buf[i] = tag[i];
        i++;
    }
    if (i < sizeof(buf) - 1) buf[i++] = ' ';
    for (size_t k = 0; k < len && i + 3 < sizeof(buf); k++) {
        buf[i++] = HEX[(b[k] >> 4) & 0xF];
        buf[i++] = HEX[b[k] & 0xF];
        if (k + 1 < len) buf[i++] = ' ';
    }
    buf[i] = '\0';
    log_msg(c, lvl, file, line, buf);
}

#define LOG(c, lvl, msg) log_msg((c), (lvl), __FILE__, __LINE__, (msg))

/* ------------------------------------------------------------------ */
/* Errors                                                              */
/* ------------------------------------------------------------------ */

const char *century_strerror(century_err_t err) {
    switch (err) {
    case CENTURY_OK: return "ok";
    case CENTURY_ERR_INVALID_ARG: return "invalid argument";
    case CENTURY_ERR_BUFFER_TOO_SMALL: return "buffer too small";
    case CENTURY_ERR_CRC: return "crc mismatch";
    case CENTURY_ERR_SHORT: return "frame too short";
    case CENTURY_ERR_ADDR_MISMATCH: return "address mismatch";
    case CENTURY_ERR_FUNC_MISMATCH: return "function mismatch";
    case CENTURY_ERR_NACK: return "nack (ack != 0x10)";
    case CENTURY_ERR_TIMEOUT: return "timeout";
    case CENTURY_ERR_IO: return "i/o error";
    case CENTURY_ERR_REPLY_MISMATCH: return "reply echo mismatch";
    default: return "unknown";
    }
}

/* ------------------------------------------------------------------ */
/* Client init                                                         */
/* ------------------------------------------------------------------ */

void century_client_init(century_client_t *c, uint8_t addr, century_transport_t *tr) {
    if (!c) return;
    c->addr = addr;
    c->tr = tr;
    c->timeout_ms = 300;
    c->gap_ms = 10;
    c->retries = 5;
    c->allow_quirk = true;
    c->log_fn = NULL;
    c->log_ctx = NULL;
    c->last_nack = 0;
}

void century_client_set_log(century_client_t *c, century_log_fn_t fn, void *ctx) {
    if (!c) return;
    c->log_fn = fn;
    c->log_ctx = ctx;
}

/* ------------------------------------------------------------------ */
/* CRC16 Modbus (poly A001, init FFFF)                                 */
/* ------------------------------------------------------------------ */

uint16_t century_crc16(const uint8_t *d, size_t len) {
    if (!d || len == 0) return 0xFFFF; /* empty input: CRC init value, NULL-safe */
#if CENTURY_CRC_TABLE
    static const uint16_t table[256] = {
        0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,
        0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
        0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,
        0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
        0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1B40,
        0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
        0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,
        0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
        0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,
        0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
        0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,
        0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
        0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,
        0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
        0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,
        0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
        0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,
        0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
        0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,
        0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
        0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,
        0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
        0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,
        0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
        0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,
        0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
        0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,
        0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
        0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,
        0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
        0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,
        0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
    };
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc = (uint16_t)((crc >> 8) ^ table[(crc ^ d[i]) & 0xFF]);
    }
    return crc;
#else
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (uint16_t)((crc >> 1) ^ 0xA001);
            else crc >>= 1;
        }
    }
    return crc;
#endif
}

/* ------------------------------------------------------------------ */
/* Build                                                               */
/* ------------------------------------------------------------------ */

int century_build_frame(uint8_t addr, uint8_t func,
                        const uint8_t *payload, size_t payload_len,
                        uint8_t *out, size_t out_cap) {
    if (!out) return CENTURY_ERR_INVALID_ARG;
    if (payload_len > CENTURY_MAX_PAYLOAD) return CENTURY_ERR_INVALID_ARG;
    if (payload_len > 0 && !payload) return CENTURY_ERR_INVALID_ARG;
    size_t need = 3 + payload_len + 2;
    if (out_cap < need) return CENTURY_ERR_BUFFER_TOO_SMALL;
    out[0] = addr;
    out[1] = func;
    out[2] = CENTURY_REQ_TAG;
    for (size_t i = 0; i < payload_len; i++) out[3 + i] = payload[i];
    uint16_t crc = century_crc16(out, 3 + payload_len);
    out[3 + payload_len] = (uint8_t)(crc & 0xFF);
    out[4 + payload_len] = (uint8_t)((crc >> 8) & 0xFF);
    return (int)need;
}

/* ------------------------------------------------------------------ */
/* Parse                                                               */
/* ------------------------------------------------------------------ */

century_err_t century_parse_frame(const uint8_t *rx, size_t rx_len,
                                  uint8_t exp_addr, uint8_t exp_func,
                                  bool allow_quirk, century_parsed_t *info) {
    if (!rx || !info) return CENTURY_ERR_INVALID_ARG;
    if (rx_len < 4) return CENTURY_ERR_SHORT;

    uint16_t want = century_crc16(rx, rx_len - 2);
    uint16_t got = (uint16_t)(rx[rx_len - 2] | ((uint16_t)rx[rx_len - 1] << 8));
    if (want != got) return CENTURY_ERR_CRC;

    if (rx[0] != exp_addr) return CENTURY_ERR_ADDR_MISMATCH;

    /* Normal form: [ADDR][FUNC][0x10][DATA][CRC] */
    if (rx_len >= 5 && rx[1] == exp_func) {
        if (rx[2] != CENTURY_ACK) return CENTURY_ERR_NACK;
        info->quirk = false;
        info->data = rx + 3;
        info->data_len = rx_len - 3 - 2;
        return CENTURY_OK;
    }
    /* Quirk form: [ADDR][0x10][DATA][CRC] */
    if (rx[1] == CENTURY_ACK) {
        if (!allow_quirk) return CENTURY_ERR_FUNC_MISMATCH;
        info->quirk = true;
        info->data = rx + 2;
        info->data_len = rx_len - 2 - 2;
        return CENTURY_OK;
    }
    if (rx_len >= 5 && rx[1] != exp_func) return CENTURY_ERR_FUNC_MISMATCH;
    return CENTURY_ERR_NACK;
}

/* Exception frame: [ADDR][FUNC|0x80][CODE][CRClo][CRChi], always 5 bytes. */
static bool is_exception_frame(const uint8_t *rx, size_t len,
                               uint8_t addr, uint8_t func, uint8_t *code_out) {
    if (len != 5) return false;
    if (rx[0] != addr || rx[1] != (uint8_t)(func | 0x80)) return false;
    uint16_t want = century_crc16(rx, 3);
    uint16_t have = (uint16_t)(rx[3] | ((uint16_t)rx[4] << 8));
    if (want != have) return false;
    *code_out = rx[2];
    return true;
}

/* ------------------------------------------------------------------ */
/* Transport exchange with retry                                       */
/* ------------------------------------------------------------------ */

static century_err_t century_request(century_client_t *c, uint8_t func,
                                     const uint8_t *pl, size_t pl_len,
                                     uint8_t *data_out, size_t data_cap,
                                     size_t *data_len_out) {
    if (!c || !c->tr || !c->tr->write || !c->tr->read) {
        return CENTURY_ERR_INVALID_ARG;
    }
    if (c->retries < 1) return CENTURY_ERR_INVALID_ARG;

    uint8_t tx[CENTURY_MAX_FRAME];
    int tx_len = century_build_frame(c->addr, func, pl, pl_len, tx, sizeof(tx));
    if (tx_len < 0) return (century_err_t)tx_len;

    c->last_nack = 0;
    log_hex(c, CENTURY_LOG_DEBUG, __FILE__, __LINE__, "TX", tx, (size_t)tx_len);

    uint8_t rx[CENTURY_MAX_FRAME];
    century_err_t last = CENTURY_ERR_TIMEOUT;

    for (int attempt = 0; attempt < c->retries; attempt++) {
        if (c->tr->set_tx_enable) c->tr->set_tx_enable(c->tr->ctx, true);
        int w = c->tr->write(c->tr->ctx, tx, (size_t)tx_len);
        if (c->tr->set_tx_enable) c->tr->set_tx_enable(c->tr->ctx, false);
        if (w < 0) {
            last = CENTURY_ERR_IO;
            LOG(c, CENTURY_LOG_DEBUG, "write failed, retrying");
            continue;
        }
        if (c->gap_ms && c->tr->delay_ms) c->tr->delay_ms(c->gap_ms);

        /* Collect response. With millis() we loop until deadline;
         * without it we do a single blocking read. */
        size_t got = 0;
        bool deadline_loop = (c->tr->millis != NULL);
        uint32_t start = deadline_loop ? c->tr->millis() : 0;

        while (1) {
            uint32_t remaining = c->timeout_ms;
            if (deadline_loop) {
                uint32_t now = c->tr->millis();
                uint32_t elapsed = now - start;
                if (elapsed >= c->timeout_ms) break;
                remaining = c->timeout_ms - elapsed;
            }
            if (got >= sizeof(rx)) break;
            int n = c->tr->read(c->tr->ctx, rx + got, sizeof(rx) - got, remaining);
            if (n < 0) {
                last = CENTURY_ERR_IO;
                got = 0;
                break;
            }
            if (n == 0) break; /* timeout slice */
            got += (size_t)n;
            /* Shortest valid frame is 4 (quirk, empty data). Keep reading
             * only if we have no millis-based deadline; with deadline keep
             * accumulating until timeout expires to catch split frames. */
            if (!deadline_loop) break;
            /* Heuristic: stop early if we already have a plausible full
             * frame (normal empty-data = 5 bytes). Longer sensor frames
             * are 7-8 bytes; waiting for the full timeout would just add
             * latency, but correctness first: if CRC already passes for a
             * prefix we accept it. Check incrementally. */
            if (got >= 4) {
                uint16_t want = century_crc16(rx, got - 2);
                uint16_t have = (uint16_t)(rx[got - 2] | ((uint16_t)rx[got - 1] << 8));
                if (want == have && got >= 5) break;
                if (want == have && got == 4 && rx[1] == CENTURY_ACK) break;
            }
        }

        if (got == 0) {
            last = (last == CENTURY_ERR_IO) ? CENTURY_ERR_IO : CENTURY_ERR_TIMEOUT;
            LOG(c, CENTURY_LOG_DEBUG, "no response, retrying");
            continue;
        }

        log_hex(c, CENTURY_LOG_DEBUG, __FILE__, __LINE__, "RX", rx, got);

        uint8_t exc = 0;
        if (is_exception_frame(rx, got, c->addr, func, &exc)) {
            c->last_nack = exc;
            LOG(c, CENTURY_LOG_WARN, century_nack_str(exc));
            return CENTURY_ERR_NACK;
        }

        century_parsed_t p;
        century_err_t pr = century_parse_frame(rx, got, c->addr, func,
                                               c->allow_quirk, &p);
        if (pr == CENTURY_OK) {
            if (p.data_len > data_cap) return CENTURY_ERR_BUFFER_TOO_SMALL;
            for (size_t i = 0; i < p.data_len; i++) data_out[i] = p.data[i];
            if (data_len_out) *data_len_out = p.data_len;
            if (p.quirk) LOG(c, CENTURY_LOG_DEBUG, "EVO quirk frame (no FUNC echo)");
            return CENTURY_OK;
        }
        if (pr == CENTURY_ERR_NACK) return pr; /* don't retry NACK */
        last = pr;
        LOG(c, CENTURY_LOG_DEBUG, "parse failed, retrying");
    }
    log_u32(c, CENTURY_LOG_WARN, __FILE__, __LINE__, "request failed func=", func);
    return last;
}

/* ------------------------------------------------------------------ */
/* High-level API                                                      */
/* ------------------------------------------------------------------ */

century_err_t century_get_status(century_client_t *c, century_status_t *out) {
    if (!c || !out) return CENTURY_ERR_INVALID_ARG;
    uint8_t data[16];
    size_t dlen = 0;
    century_err_t e = century_request(c, CENTURY_FUNC_STATUS, NULL, 0,
                                      data, sizeof(data), &dlen);
    if (e != CENTURY_OK) return e;
    if (dlen < 1) return CENTURY_ERR_SHORT;
    switch (data[0]) {
    case CENTURY_STATUS_STOPPED:
    case CENTURY_STATUS_BOOTING:
    case CENTURY_STATUS_ACCELERATING:
    case CENTURY_STATUS_RUNNING:
    case CENTURY_STATUS_FAULT:
        *out = (century_status_t)data[0];
        break;
    default:
        *out = CENTURY_STATUS_UNKNOWN;
        break;
    }
    return CENTURY_OK;
}

const char *century_status_str(century_status_t s) {
    switch (s) {
    case CENTURY_STATUS_STOPPED: return "stopped";
    case CENTURY_STATUS_BOOTING: return "booting";
    case CENTURY_STATUS_ACCELERATING: return "accelerating";
    case CENTURY_STATUS_RUNNING: return "running";
    case CENTURY_STATUS_FAULT: return "fault";
    default: return "unknown";
    }
}

century_err_t century_run(century_client_t *c) {
    if (!c) return CENTURY_ERR_INVALID_ARG;
    uint8_t data[16];
    size_t dlen = 0;
    return century_request(c, CENTURY_FUNC_RUN, NULL, 0, data, sizeof(data), &dlen);
}

century_err_t century_stop(century_client_t *c) {
    if (!c) return CENTURY_ERR_INVALID_ARG;
    uint8_t data[16];
    size_t dlen = 0;
    return century_request(c, CENTURY_FUNC_STOP, NULL, 0, data, sizeof(data), &dlen);
}

/* Clamp + round to 50-step, WARN on any change. */
static uint16_t normalize_rpm(const century_client_t *c, uint16_t rpm) {
    uint16_t orig = rpm;
    if (rpm < CENTURY_MIN_RPM) rpm = CENTURY_MIN_RPM;
    if (rpm > CENTURY_MAX_RPM) rpm = CENTURY_MAX_RPM;
    uint16_t rounded = (uint16_t)(((rpm + CENTURY_RPM_STEP / 2) / CENTURY_RPM_STEP) * CENTURY_RPM_STEP);
    if (rounded < CENTURY_MIN_RPM) rounded = CENTURY_MIN_RPM;
    if (rounded > CENTURY_MAX_RPM) rounded = CENTURY_MAX_RPM;
    if (rounded != orig) {
        char buf[CENTURY_LOG_MSG_MAX];
        size_t i = 0;
        const char *pre = "rpm ";
        while (*pre && i < sizeof(buf) - 24) buf[i++] = *pre++;
        i += fmt_u32(buf + i, sizeof(buf) - 1 - i, orig);
        const char *mid = " -> ";
        while (*mid && i < sizeof(buf) - 12) buf[i++] = *mid++;
        i += fmt_u32(buf + i, sizeof(buf) - 1 - i, rounded);
        buf[i] = '\0';
        log_msg(c, CENTURY_LOG_WARN, __FILE__, __LINE__, buf);
    }
    return rounded;
}

century_err_t century_set_demand(century_client_t *c, century_mode_t mode, uint16_t rpm) {
    if (!c) return CENTURY_ERR_INVALID_ARG;
    if (mode != CENTURY_MODE_SPEED && mode != CENTURY_MODE_TORQUE) {
        return CENTURY_ERR_INVALID_ARG;
    }
    uint16_t use = normalize_rpm(c, rpm);
    uint16_t raw = (uint16_t)(use * 4u);
    uint8_t pl[3];
    pl[0] = (uint8_t)mode;
    pl[1] = (uint8_t)(raw & 0xFF);
    pl[2] = (uint8_t)((raw >> 8) & 0xFF);
    uint8_t data[16];
    size_t dlen = 0;
    century_err_t e = century_request(c, CENTURY_FUNC_SET_DEMAND, pl, sizeof(pl),
                                      data, sizeof(data), &dlen);
    if (e != CENTURY_OK) return e;
    /* Motor echoes [MODE, DEMANDlo, DEMANDhi]; reject mismatches. */
    if (dlen != 3 || data[0] != (uint8_t)mode ||
        data[1] != pl[1] || data[2] != pl[2]) {
        return CENTURY_ERR_REPLY_MISMATCH;
    }
    return CENTURY_OK;
}

century_err_t century_set_demand_and_run(century_client_t *c, uint16_t rpm) {
    if (!c) return CENTURY_ERR_INVALID_ARG;
    century_err_t e = century_set_demand(c, CENTURY_MODE_SPEED, rpm);
    if (e != CENTURY_OK) return e;
    return century_run(c);
}

century_err_t century_read_sensor(century_client_t *c, uint8_t page, uint8_t addr,
                                  uint16_t scale, century_sensor_t *out) {
    if (!c || !out) return CENTURY_ERR_INVALID_ARG;
    uint8_t pl[2] = { page, addr };
    uint8_t data[16];
    size_t dlen = 0;
    century_err_t e = century_request(c, CENTURY_FUNC_READ_SENSOR, pl, sizeof(pl),
                                      data, sizeof(data), &dlen);
    if (e != CENTURY_OK) return e;
    if (dlen != 3 && dlen != 4) return CENTURY_ERR_SHORT;
    if (data[0] != page || data[1] != addr) return CENTURY_ERR_REPLY_MISMATCH;
    out->page_echo = data[0];
    out->addr_echo = data[1];
    uint16_t raw;
    if (dlen == 3) {
        raw = data[2]; /* 8-bit form */
    } else {
        raw = (uint16_t)(data[2] | ((uint16_t)data[3] << 8));
    }
    out->raw = raw;
    out->unsupported = (dlen == 4 && raw == 0xFFFFu);
    out->scaled = (scale == 0) ? raw : (uint16_t)(raw / scale);
    return CENTURY_OK;
}

century_err_t century_read_rpm(century_client_t *c, uint16_t *rpm_out) {
    if (!rpm_out) return CENTURY_ERR_INVALID_ARG;
    century_sensor_t s;
    century_err_t e = century_read_sensor(c, 0, 0, 4, &s);
    if (e != CENTURY_OK) return e;
    *rpm_out = s.scaled;
    return CENTURY_OK;
}

century_err_t century_read_demand_rpm(century_client_t *c, uint16_t *rpm_out) {
    if (!rpm_out) return CENTURY_ERR_INVALID_ARG;
    century_sensor_t s;
    century_err_t e = century_read_sensor(c, 0, 3, 4, &s);
    if (e != CENTURY_OK) return e;
    *rpm_out = s.scaled;
    return CENTURY_OK;
}

century_err_t century_read_power_watts(century_client_t *c, uint16_t *watts_out) {
    if (!watts_out) return CENTURY_ERR_INVALID_ARG;
    century_sensor_t s;
    century_err_t e = century_read_sensor(c, CENTURY_SENS_WATTS_PAGE,
                                          CENTURY_SENS_WATTS_ADDR, 1, &s);
    if (e != CENTURY_OK) return e;
    *watts_out = s.scaled;
    return CENTURY_OK;
}

century_err_t century_read_ambient_temp_raw(century_client_t *c, int16_t *raw_out) {
    if (!raw_out) return CENTURY_ERR_INVALID_ARG;
    century_sensor_t s;
    century_err_t e = century_read_sensor(c, CENTURY_SENS_TEMP_PAGE,
                                          CENTURY_SENS_TEMP_ADDR, 1, &s);
    if (e != CENTURY_OK) return e;
    *raw_out = (int16_t)s.raw; /* signed, 1 LSB = 1/128 degC */
    return CENTURY_OK;
}

century_err_t century_read_fault_code(century_client_t *c, uint8_t page, uint8_t addr,
                                      uint8_t *code_out) {
    if (!code_out) return CENTURY_ERR_INVALID_ARG;
    century_sensor_t s;
    century_err_t e = century_read_sensor(c, page, addr, 1, &s);
    if (e != CENTURY_OK) return e;
    *code_out = (uint8_t)(s.raw & 0xFF); /* code is the low byte */
    return CENTURY_OK;
}

const char *century_fault_str(uint8_t code) {
    switch (code) {
    case 0x00: return "None";
    case 0x21: return "Software Overcurrent";
    case 0x22: return "DC Overvoltage";
    case 0x23: return "DC Undervoltage";
    case 0x26: return "Hardware Overcurrent";
    case 0x2A: return "Startup Failure";
    case 0x2D: return "Processor - Fatal";
    case 0x2E: return "IGBT Over Temperature";
    case 0x2F: return "Loss of Phase";
    case 0x30: return "Low Power";
    case 0x31: return "Processor - Registers";
    case 0x32: return "Processor - Program Counter";
    case 0x33: return "Processor - Interrupt/Execution";
    case 0x34: return "Processor - Clock";
    case 0x35: return "Processor - Flash Memory";
    case 0x36: return "RAS Fault";
    case 0x37: return "Processor - ADC";
    case 0x3C: return "Keypad Fault";
    case 0x3D: return "LVB Data Flash Fault";
    case 0x3E: return "Comm Loss Fault - LVB & Drive";
    case 0x3F: return "Generic Fault";
    case 0x40: return "Coherence Fault";
    case 0x41: return "UL Fault";
    case 0x42: return "SVRS Fault Type 1";
    case 0x43: return "SVRS Fault Type 2";
    case 0x44: return "SVRS Fault Type 13";
    default: return "Unknown";
    }
}

century_err_t century_set_aux_relay(century_client_t *c, bool on) {
    if (!c) return CENTURY_ERR_INVALID_ARG;
    uint8_t pl[4] = {
        CENTURY_CFG_PAGE1_WRITE, CENTURY_CFG_AUX_RELAY_ADDR, 0x00,
        on ? CENTURY_AUX_ON_VALUE : CENTURY_AUX_OFF_VALUE
    };
    uint8_t data[16];
    size_t dlen = 0;
    century_err_t e = century_request(c, CENTURY_FUNC_CONFIG_RW, pl, sizeof(pl),
                                      data, sizeof(data), &dlen);
    if (e != CENTURY_OK) return e;
    if (dlen != 4) return CENTURY_ERR_SHORT;
    return CENTURY_OK;
}

century_err_t century_store_config(century_client_t *c) {
    if (!c) return CENTURY_ERR_INVALID_ARG;
    uint8_t data[16];
    size_t dlen = 0;
    return century_request(c, CENTURY_FUNC_STORE_CONFIG, NULL, 0,
                           data, sizeof(data), &dlen);
}

uint8_t century_last_nack(const century_client_t *c) {
    return c ? c->last_nack : 0;
}

const char *century_nack_str(uint8_t code) {
    switch (code) {
    case 0x00: return "OK (no NACK)";
    case 0x01: return "Illegal command";
    case 0x02: return "Operand out of range";
    case 0x03: return "Data out of range";
    case 0x04: return "General failure / fault mode";
    case 0x05: return "Incorrect command length";
    case 0x06: return "Command cannot execute now";
    case 0x09: return "Buffer error";
    case 0x0A: return "Running parameters incomplete";
    default: return "Unknown NACK";
    }
}

/* ------------------------------------------------------------------ */
/* Scan / auto-detect                                                  */
/* ------------------------------------------------------------------ */

century_err_t century_scan(century_client_t *c, uint8_t start, uint8_t end,
                           uint32_t per_addr_timeout_ms,
                           century_scan_hit_t *out, size_t cap, size_t *found) {
    if (!c || !out || !found) return CENTURY_ERR_INVALID_ARG;
    if (start < CENTURY_ADDR_MIN) start = CENTURY_ADDR_MIN;
    if (end > CENTURY_ADDR_MAX) end = CENTURY_ADDR_MAX;
    if (start > end) return CENTURY_ERR_INVALID_ARG;

    uint8_t saved_addr = c->addr;
    uint32_t saved_timeout = c->timeout_ms;
    int saved_retries = c->retries;
    century_log_fn_t saved_log = c->log_fn;
    void *saved_log_ctx = c->log_ctx;

    size_t n = 0;
    c->timeout_ms = per_addr_timeout_ms ? per_addr_timeout_ms : 150;
    c->retries = 1; /* fast sweep: one attempt per address */
    /* Silence per-address misses: 246/247 addrs normally time out.
     * Without this each miss logs WARN "request failed func=" (spam). */
    c->log_fn = NULL;
    c->log_ctx = NULL;

    for (uint16_t a = start; a <= end; a++) {
        c->addr = (uint8_t)a;
        uint8_t data[16];
        size_t dlen = 0;
        if (century_request(c, CENTURY_FUNC_STATUS, NULL, 0,
                            data, sizeof(data), &dlen) == CENTURY_OK && dlen >= 1) {
            if (n < cap) {
                out[n].addr = (uint8_t)a;
                out[n].status_byte = data[0];
            }
            /* Keep counting past cap so *found is the true total. */
            n++;
        }
    }

    c->addr = saved_addr;
    c->timeout_ms = saved_timeout;
    c->retries = saved_retries;
    c->log_fn = saved_log;
    c->log_ctx = saved_log_ctx;
    for (size_t i = 0; i < n && i < cap; i++) {
        log_u32(c, CENTURY_LOG_INFO, __FILE__, __LINE__, "found motor at addr=", out[i].addr);
    }
    *found = n;
    return CENTURY_OK;
}

century_err_t century_auto_detect(century_client_t *c, uint8_t *addr_out) {
    if (!c || !addr_out) return CENTURY_ERR_INVALID_ARG;
    century_scan_hit_t hits[8];
    size_t found = 0;
    century_err_t e = century_scan(c, CENTURY_ADDR_MIN, CENTURY_ADDR_MAX,
                                   150, hits, 8, &found);
    if (e != CENTURY_OK) return e;
    if (found == 0) return CENTURY_ERR_TIMEOUT;
    if (found > 1) {
        LOG(c, CENTURY_LOG_WARN, "multiple motors found, using first");
    }
    *addr_out = hits[0].addr;
    c->addr = hits[0].addr;
    return CENTURY_OK;
}
