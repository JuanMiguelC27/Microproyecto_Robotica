#include "battery.h"
#include "config.h"

namespace bat {

static const uint8_t PINS[3] = {cfg::PIN_VBAT_CTRL, cfg::PIN_VBAT_TRAC, cfg::PIN_VBAT_PROP};

void begin() {
  for (uint8_t i = 0; i < 3; i++) analogSetPinAttenuation(PINS[i], ADC_11db);   // rango hasta ~2,45 V lineal
}

uint8_t pin(Id id) { return PINS[id]; }
float divider(Id id) { return id == CTRL ? cfg::DIV_CTRL : id == TRAC ? cfg::DIV_TRAC : cfg::DIV_PROP; }
float calibration(Id id) { return id == CTRL ? cfg::CAL_CTRL : id == TRAC ? cfg::CAL_TRAC : cfg::CAL_PROP; }
const char* name(Id id) {
  return id == CTRL ? "2S control (J2, D35)" : id == TRAC ? "2S traccion (J6, VN/D39)" : "3S EDF (PCB-C, VP/D36)";
}

Reading read(Id id, uint8_t samples) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) sum += analogReadMilliVolts(PINS[id]);
  Reading r;
  r.pin_mV = sum / samples;
  // El ADC de la ESP32 con 11 dB no baja de ~140 mV aunque el pin este en 0 V (piso de la calibracion).
  // Por debajo de ADC_FLOOR_MV se toma como 0 V: sin bateria se ve 0.00 V en vez de ~0.5 V.
  r.volts = r.pin_mV < cfg::ADC_FLOOR_MV ? 0.0f : r.pin_mV / 1000.0f * divider(id) * calibration(id);
  r.present = r.volts >= cfg::V_PRESENT;
  if (!r.present) {
    r.state = "NO CONECTADA";
  } else if (id == PROP) {
    r.state = r.volts > cfg::V3S_FULL + 0.3f ? "SOBRE TENSION" : r.volts >= cfg::V3S_FULL - 0.3f ? "LLENA"
            : r.volts >= cfg::V3S_LOW ? "OK" : r.volts >= cfg::V3S_CRIT ? "BAJA" : "CRITICA";
  } else {
    r.state = r.volts > cfg::V2S_FULL + 0.3f ? "SOBRE TENSION" : r.volts >= cfg::V2S_FULL - 0.2f ? "LLENA"
            : r.volts >= cfg::V2S_LOW ? "OK" : r.volts >= cfg::V2S_MIN ? "BAJA" : "CRITICA";
  }
  return r;
}

}  // namespace bat
