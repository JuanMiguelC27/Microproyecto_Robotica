#include "ibus.h"

void IBus::begin(HardwareSerial& port, uint8_t rxPin, uint32_t baud) {
  port_ = &port;
  port_->begin(baud, SERIAL_8N1, rxPin, -1);   // solo recepcion: no se usa ningun pin de TX
}

void IBus::update() {
  if (!port_) return;
  while (port_->available() > 0) feed((uint8_t)port_->read());
}

void IBus::feed(uint8_t b) {
  bytes_++;
  if (idx_ == 0 && b != 0x20) return;            // esperar el inicio de trama
  if (idx_ == 1 && b != 0x40) {                  // segundo byte equivocado: resincronizar
    idx_ = (b == 0x20) ? 1 : 0;
    if (idx_ == 1) buf_[0] = b;
    return;
  }
  buf_[idx_++] = b;
  if (idx_ < 32) return;
  idx_ = 0;
  uint16_t sum = 0xFFFF;
  for (uint8_t i = 0; i < 30; i++) sum -= buf_[i];
  uint16_t rx = (uint16_t)buf_[30] | ((uint16_t)buf_[31] << 8);
  if (sum != rx) { bad_++; return; }
  for (uint8_t c = 0; c < CHANNELS; c++) ch_[c] = (uint16_t)buf_[2 + 2 * c] | ((uint16_t)(buf_[3 + 2 * c] & 0x0F) << 8);
  lastMs_ = millis();
  any_ = true;
  ok_++;
}

bool IBus::fresh(uint32_t maxAgeMs) const { return any_ && (millis() - lastMs_) < maxAgeMs; }
uint32_t IBus::ageMs() const { return any_ ? millis() - lastMs_ : 0xFFFFFFFFUL; }
