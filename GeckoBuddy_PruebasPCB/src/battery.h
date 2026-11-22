// Lectura de las tres baterias por los divisores de la placa.
#pragma once
#include <Arduino.h>

namespace bat {

enum Id : uint8_t { CTRL = 0, TRAC = 1, PROP = 2 };

struct Reading {
  uint32_t pin_mV;      // tension en el pin del ADC
  float volts;          // tension de la bateria (con divisor y calibracion)
  bool present;         // false si esta por debajo de cfg::V_PRESENT
  const char* state;    // "LLENA", "OK", "BAJA", "CRITICA", "NO CONECTADA", "SOBRE TENSION"
};

void begin();
Reading read(Id id, uint8_t samples = 16);
const char* name(Id id);
float divider(Id id);
float calibration(Id id);
uint8_t pin(Id id);

}  // namespace bat
