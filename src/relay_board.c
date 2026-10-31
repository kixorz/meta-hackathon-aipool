#include "relay_board.h"

uint16_t relay_board_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    if (!data) return crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1u)
                crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            else
                crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

void relay_board_append_crc(uint8_t *frame, size_t len_wo_crc)
{
    uint16_t crc;
    if (!frame) return;
    crc = relay_board_crc16(frame, len_wo_crc);
    frame[len_wo_crc]     = (uint8_t)(crc & 0xFFu); /* low byte first */
    frame[len_wo_crc + 1] = (uint8_t)((crc >> 8) & 0xFFu);
}

bool relay_board_check_crc(const uint8_t *frame, size_t len)
{
    uint16_t got, calc;
    if (!frame || len < 3) return false;
    got  = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    calc = relay_board_crc16(frame, len - 2);
    return got == calc;
}

size_t relay_board_build_read_coils(uint8_t slave, uint8_t *out)
{
    if (!out) return 0;
    out[0] = slave;
    out[1] = RELAY_BOARD_FUNC_READ_COILS;
    out[2] = 0x00;
    out[3] = 0x00;
    out[4] = 0x00;
    out[5] = 0x08;
    relay_board_append_crc(out, 6);
    return RELAY_BOARD_READ_REQ_LEN;
}

size_t relay_board_build_read_inputs(uint8_t slave, uint8_t *out)
{
    if (!out) return 0;
    out[0] = slave;
    out[1] = RELAY_BOARD_FUNC_READ_INPUTS;
    out[2] = 0x00;
    out[3] = 0x00;
    out[4] = 0x00;
    out[5] = 0x08;
    relay_board_append_crc(out, 6);
    return RELAY_BOARD_READ_REQ_LEN;
}

size_t relay_board_build_write_coil(uint8_t slave, uint8_t relay_1_8,
                                    bool on, uint8_t *out)
{
    uint16_t coil;
    if (!out || !RELAY_BOARD_IS_RELAY(relay_1_8)) return 0;
    coil = RELAY_BOARD_COIL(relay_1_8);
    out[0] = slave;
    out[1] = RELAY_BOARD_FUNC_WRITE_COIL;
    out[2] = (uint8_t)(coil >> 8);
    out[3] = (uint8_t)(coil & 0xFFu);
    out[4] = on ? 0xFFu : 0x00u;
    out[5] = 0x00u;
    relay_board_append_crc(out, 6);
    return RELAY_BOARD_WRITE_REQ_LEN;
}

size_t relay_board_build_write_all(uint8_t slave, uint8_t bitmap,
                                   uint8_t *out)
{
    if (!out) return 0;
    out[0] = slave;
    out[1] = RELAY_BOARD_FUNC_WRITE_COILS;
    out[2] = 0x00;
    out[3] = 0x00;
    out[4] = 0x00;
    out[5] = 0x08;
    out[6] = 0x01;
    out[7] = bitmap;
    relay_board_append_crc(out, 8);
    return RELAY_BOARD_WRITE_ALL_REQ_LEN;
}

bool relay_board_parse_coils(const uint8_t *rx, size_t len,
                             uint8_t slave, uint8_t *bitmap)
{
    if (!rx || !bitmap) return false;
    if (len != RELAY_BOARD_READ_COILS_RESP_LEN) return false;
    if (rx[0] != slave) return false;
    if (rx[1] != RELAY_BOARD_FUNC_READ_COILS) return false;
    if (rx[2] != 0x01) return false;
    if (!relay_board_check_crc(rx, len)) return false;
    *bitmap = rx[3];
    return true;
}

bool relay_board_parse_inputs(const uint8_t *rx, size_t len,
                              uint8_t slave, uint8_t *bitmap)
{
    if (!rx || !bitmap) return false;
    if (len != RELAY_BOARD_READ_INPUTS_RESP_LEN) return false;
    if (rx[0] != slave) return false;
    if (rx[1] != RELAY_BOARD_FUNC_READ_INPUTS) return false;
    if (rx[2] != 0x01) return false;
    if (!relay_board_check_crc(rx, len)) return false;
    *bitmap = rx[3];
    return true;
}

bool relay_board_is_write_echo(const uint8_t *tx, const uint8_t *rx,
                               size_t len)
{
    size_t i;
    if (!tx || !rx) return false;
    if (len != RELAY_BOARD_WRITE_RESP_LEN) return false;
    for (i = 0; i < len; i++) {
        if (tx[i] != rx[i]) return false;
    }
    return relay_board_check_crc(rx, len);
}

bool relay_board_get(uint8_t bitmap, uint8_t relay_1_8)
{
    if (!RELAY_BOARD_IS_RELAY(relay_1_8)) return false;
    return (bitmap & RELAY_BOARD_BIT(relay_1_8)) != 0;
}
