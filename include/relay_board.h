#ifndef RELAY_BOARD_H
#define RELAY_BOARD_H

/*
 * RS485 8-Channel Relay Board — Modbus RTU driver.
 *
 * Probed hardware: generic 8-ch Modbus RTU coil board, slave 1,
 * 9600 baud 8N1. Coils 0x0000-0x0007 = relays 1-8.
 * Supported: 0x01 Read Coils, 0x02 Read Discrete Inputs,
 *            0x05 Write Single Coil.
 * Unsupported (silent): 0x03/0x04/0x06 (incl. R421A08 custom map).
 * Unverified: 0x0F Write Multiple Coils (builder provided).
 *
 * No UART / OS dependency. You provide transport; this builds,
 * checks, and parses raw Modbus RTU frames.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Config --- */
#define RELAY_BOARD_SLAVE_ADDR  0x01u
#define RELAY_BOARD_BAUD        9600
#define RELAY_BOARD_NUM_RELAYS  8u

#define RELAY_BOARD_FUNC_READ_COILS   0x01u
#define RELAY_BOARD_FUNC_READ_INPUTS  0x02u
#define RELAY_BOARD_FUNC_WRITE_COIL   0x05u
#define RELAY_BOARD_FUNC_WRITE_COILS  0x0Fu /* unverified on this board */

#define RELAY_BOARD_READ_REQ_LEN        8u  /* incl. 2-byte CRC */
#define RELAY_BOARD_WRITE_REQ_LEN       8u  /* incl. 2-byte CRC */
#define RELAY_BOARD_WRITE_ALL_REQ_LEN   10u /* 0x0F, incl. 2-byte CRC */
#define RELAY_BOARD_READ_COILS_RESP_LEN  6u  /* 01 01 01 <bitmap> <crc> */
#define RELAY_BOARD_READ_INPUTS_RESP_LEN 6u  /* 01 02 01 <bitmap> <crc> */
#define RELAY_BOARD_WRITE_RESP_LEN      8u  /* echo of write request */

/* Relay numbers are 1..8; coil addresses are 0..7. */
#define RELAY_BOARD_COIL(n)     ((uint16_t)((n) - 1u))
#define RELAY_BOARD_BIT(n)      ((uint8_t)(1u << ((n) - 1u)))
#define RELAY_BOARD_IS_RELAY(n) ((n) >= 1u && (n) <= 8u)

/* --- CRC-16/Modbus (poly 0xA001, init 0xFFFF, low byte first) --- */
uint16_t relay_board_crc16(const uint8_t *data, size_t len);
void relay_board_append_crc(uint8_t *frame, size_t len_wo_crc);
bool relay_board_check_crc(const uint8_t *frame, size_t len);

/* --- Frame builders (return bytes written, 0 on bad arg) --- */

/* 01 01 00 00 00 08 3D CC */
size_t relay_board_build_read_coils(uint8_t slave, uint8_t *out);

/* 01 02 00 00 00 08 79 CC */
size_t relay_board_build_read_inputs(uint8_t slave, uint8_t *out);

/*
 * ON:  01 05 00 <coil> FF 00 + CRC (ch1: 01 05 00 00 FF 00 8C 3A)
 * OFF: 01 05 00 <coil> 00 00 + CRC (ch1: 01 05 00 00 00 00 CD CA)
 */
size_t relay_board_build_write_coil(uint8_t slave, uint8_t relay_1_8,
                                    bool on, uint8_t *out);

/* 0x0F unverified: 01 0F 00 00 00 08 01 <bitmap> + CRC */
size_t relay_board_build_write_all(uint8_t slave, uint8_t bitmap,
                                   uint8_t *out);

/* --- Response parsers --- */

/* Expects 01 01 01 <bitmap> <crcLo> <crcHi>. Sets *bitmap on success. */
bool relay_board_parse_coils(const uint8_t *rx, size_t len,
                             uint8_t slave, uint8_t *bitmap);

/* Expects 01 02 01 <bitmap> <crcLo> <crcHi>. Sets *bitmap on success. */
bool relay_board_parse_inputs(const uint8_t *rx, size_t len,
                              uint8_t slave, uint8_t *bitmap);

/* Write Single Coil echoes the request. True if rx == tx and CRC OK. */
bool relay_board_is_write_echo(const uint8_t *tx, const uint8_t *rx,
                               size_t len);

/* True if relay_1_8 bit is set in bitmap. False on bad relay number. */
bool relay_board_get(uint8_t bitmap, uint8_t relay_1_8);

#ifdef __cplusplus
}
#endif

#endif /* RELAY_BOARD_H */
