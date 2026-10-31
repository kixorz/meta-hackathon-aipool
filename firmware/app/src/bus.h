#pragma once

// Shared RS485 bus (relay board + Century pump, one UART2).
// Values from docs/aipool-spec.md + tests/esp32-combined-test wiring.
// Phase 0: mutex + UART handle only. Phase 1 adds pump/valve txns.

#include <Arduino.h>

// Strapping-safe: keep TX off GPIO12 (MTDI). GPIO14 is safe.
#define AIPOOL_RS485_TX_PIN 14
#define AIPOOL_RS485_RX_PIN 13
#define AIPOOL_RS485_BAUD 9600
#define AIPOOL_RS485_DE_PIN -1 // auto-direction transceiver; set GPIO# if manual DE/RE needed

extern HardwareSerial RS485Bus; // UART2
extern SemaphoreHandle_t g_busMutex;

inline void busInit() {
  if (g_busMutex == nullptr) {
    g_busMutex = xSemaphoreCreateMutex();
  }
#if AIPOOL_RS485_DE_PIN >= 0
  pinMode(AIPOOL_RS485_DE_PIN, OUTPUT);
  digitalWrite(AIPOOL_RS485_DE_PIN, LOW);
#endif
  RS485Bus.begin(AIPOOL_RS485_BAUD, SERIAL_8N1, AIPOOL_RS485_RX_PIN,
                 AIPOOL_RS485_TX_PIN);
}

// Take with timeout (ms). Returns true if acquired.
inline bool busTake(uint32_t timeoutMs = 1000) {
  if (g_busMutex == nullptr)
    return false;
  return xSemaphoreTake(g_busMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

inline void busGive() {
  if (g_busMutex != nullptr) {
    xSemaphoreGive(g_busMutex);
  }
}
