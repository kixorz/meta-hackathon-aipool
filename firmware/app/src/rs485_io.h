#pragma once

// Shared Modbus-RTU transaction over the single RS485 bus.
// Serializes with g_busMutex (bus.h). Resyncs RX before TX, enforces
// >3.5-char silence, then reads until 20ms inter-byte gap or timeout.

#include <Arduino.h>
#include "bus.h"

#define AIPOOL_MODBUS_TIMEOUT_MS 300
#define AIPOOL_MODBUS_GAP_MS 5
#define AIPOOL_MODBUS_IDLE_MS 20

inline size_t rs485Txn(const uint8_t *tx, size_t txLen, uint8_t *rx,
                        size_t rxCap,
                        uint32_t timeoutMs = AIPOOL_MODBUS_TIMEOUT_MS) {
  if (!tx || txLen == 0 || !rx || rxCap == 0)
    return 0;
  if (!busTake(1500))
    return 0;

  while (RS485Bus.available())
    (void)RS485Bus.read(); // resync

#if AIPOOL_RS485_DE_PIN >= 0
  digitalWrite(AIPOOL_RS485_DE_PIN, HIGH);
  delayMicroseconds(200);
#endif
  RS485Bus.write(tx, txLen);
  RS485Bus.flush();
  delay(AIPOOL_MODBUS_GAP_MS);
#if AIPOOL_RS485_DE_PIN >= 0
  digitalWrite(AIPOOL_RS485_DE_PIN, LOW);
#endif

  size_t n = 0;
  unsigned long t0 = millis(), lastByte = t0;
  while (millis() - t0 < timeoutMs) {
    while (RS485Bus.available()) {
      int c = RS485Bus.read();
      if (c < 0)
        break;
      if (n < rxCap)
        rx[n++] = (uint8_t)c;
      lastByte = millis();
    }
    if (n > 0 && millis() - lastByte > AIPOOL_MODBUS_IDLE_MS)
      break; // end of frame
    delay(1);
  }
  busGive();
  return n;
}
