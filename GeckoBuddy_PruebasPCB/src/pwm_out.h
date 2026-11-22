// Envoltura de LEDC que compila con Arduino-ESP32 2.x y 3.x.
#pragma once
#include <Arduino.h>
#if defined(__has_include)
#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

class PwmOut {
 public:
  void begin(uint8_t pin, uint8_t channel, uint32_t freq, uint8_t bits) {
    pin_ = pin; ch_ = channel; bits_ = bits; freq_ = freq;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(pin_, freq_, bits_, ch_);
#else
    ledcSetup(ch_, freq_, bits_);
    ledcAttachPin(pin_, ch_);
#endif
    writeRaw(0);
  }
  void writeRaw(uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin_, duty);
#else
    ledcWrite(ch_, duty);
#endif
    duty_ = duty;
  }
  uint32_t maxDuty() const { return (1UL << bits_) - 1; }
  void writeFraction(float f) {               // 0..1
    if (f < 0) f = 0;
    if (f > 1) f = 1;
    writeRaw((uint32_t)(f * maxDuty() + 0.5f));
  }
  void writeMicros(int us) {                  // para 50 Hz: periodo de 20000 us
    uint32_t period = 1000000UL / freq_;
    writeRaw((uint32_t)((uint64_t)us * (maxDuty() + 1) / period));
  }
  uint32_t duty() const { return duty_; }
  uint8_t pin() const { return pin_; }

 private:
  uint8_t pin_ = 0, ch_ = 0, bits_ = 8;
  uint32_t freq_ = 1000, duty_ = 0;
};
