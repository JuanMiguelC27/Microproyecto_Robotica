// Receptor FlySky por i-BUS: UART2 a 115200, solo RX en D34.
// Trama de 32 bytes: 0x20 0x40, 14 canales (2 bytes, little endian), suma de 16 bits = 0xFFFF - suma de los 30 primeros.
#pragma once
#include <Arduino.h>

class IBus {
 public:
  static const uint8_t CHANNELS = 14;
  void begin(HardwareSerial& port, uint8_t rxPin, uint32_t baud);
  void update();                                // llamar seguido; lee todo lo que haya llegado
  uint16_t ch(uint8_t i) const { return i < CHANNELS ? ch_[i] : 0; }   // i = 0 -> CH1
  bool fresh(uint32_t maxAgeMs) const;          // hubo una trama valida hace menos de maxAgeMs
  uint32_t ageMs() const;
  uint32_t framesOk() const { return ok_; }
  uint32_t framesBad() const { return bad_; }
  uint32_t bytesIn() const { return bytes_; }
  // para las pruebas en el PC: procesa un byte como si llegara por el puerto
  void feed(uint8_t b);

 private:
  HardwareSerial* port_ = nullptr;
  uint8_t buf_[32];
  uint8_t idx_ = 0;
  uint16_t ch_[CHANNELS] = {0};
  uint32_t lastMs_ = 0;
  bool any_ = false;
  uint32_t ok_ = 0, bad_ = 0, bytes_ = 0;
};
