// Gecko Buddy - configuracion de las pruebas. Todo lo que depende del hardware esta aqui.
#pragma once
#include <Arduino.h>

namespace cfg {

// ---------- Motores (TB6612FNG, canales A y B en paralelo en cada modulo) ----------
struct MotorPins {
  uint8_t pwm, in1, in2;
  bool invert;            // los motores del lado derecho van en espejo: "adelante" del robot es el giro contrario
  bool right;             // lado del robot (visto desde atras): false = izquierdo, true = derecho
  const char* name;
};
// Numeracion: M1 y M2 adelante, M3 y M4 atras; impares a la izquierda, pares a la derecha.
static const MotorPins MOTORS[4] = {
  {25, 32, 33, false, false, "M1 delantero izquierdo (U1, J8)"},
  {14, 26, 27, true,  true,  "M2 delantero derecho (U2, J9)"},
  {21, 22, 23, false, false, "M3 trasero izquierdo (U3, J10)"},  // en tu PCB: IN1 = D22, IN2 = D23
  {15,  4,  2, true,  true,  "M4 trasero derecho (U4, J11)"},
};

// ---------- Otros pines ----------
static const uint8_t PIN_STBY       = 13;  // STBY de los 4 TB6612 + LED de estado
static const uint8_t PIN_GND_BRIDGE = 12;  // D12 UNIDO A GND EN LA PCB: solo entrada, NUNCA salida
static const uint8_t PIN_IBUS_RX    = 34;  // i-BUS SERVO del FS-iA6B (J1-3)
static const uint8_t PIN_VBAT_CTRL  = 35;  // 2S de control (J2, R1/R2)
static const uint8_t PIN_VBAT_TRAC  = 39;  // 2S de traccion (J6, R3/R4) - pin VN
static const uint8_t PIN_VBAT_PROP  = 36;  // 3S de las EDF (PCB-C: XT60 J7, R1/R2) - pin VP
static const uint8_t PIN_ESC1       = 17;  // TX2 -> J4-3 -> PCB-C J1-3 -> J3
static const uint8_t PIN_ESC2       = 16;  // RX2 -> J4-4 -> PCB-C J1-4 -> J4
static const uint8_t PIN_SERVO_L    = 19;  // J4-1 -> PCB-C J1-1 -> J5
static const uint8_t PIN_SERVO_R    = 18;  // J4-2 -> PCB-C J1-2 -> J6

// ---------- Divisores: Vbat = Vpin * (Rarriba + Rabajo) / Rabajo ----------
static const float DIV_CTRL = (100.0f + 39.0f) / 39.0f;   // 100k / 39k (OJO: tu KiCad dice R2 = 33k; si sueldas 33k usa (100+33)/33)
static const float DIV_TRAC = (100.0f + 39.0f) / 39.0f;   // 100k / 39k
static const float DIV_PROP = (100.0f + 22.0f) / 22.0f;   // PCB-C: R1 100k / R2 22k
// Calibracion: la prueba 2 te da el valor exacto despues de medir con el multimetro.
static const uint32_t ADC_FLOOR_MV = 200;   // lectura minima del ADC con el pin a 0 V (~140 mV): debajo = 0 V
static const float CAL_CTRL = 1.000f;
static const float CAL_TRAC = 1.000f;
static const float CAL_PROP = 1.000f;

// ---------- Limites de las baterias (V) ----------
static const float V2S_FULL = 8.40f, V2S_LOW = 6.60f, V2S_MIN = 6.00f;
static const float V3S_FULL = 12.60f, V3S_LOW = 10.50f, V3S_CRIT = 9.90f;
static const float V_PRESENT = 3.0f;      // por debajo: bateria no conectada

// ---------- Motores ----------
static const float    V_MOTOR        = 6.0f;   // NovaMax de 6 V: tope de PWM = 6,0 / VBAT_TRAC
static const uint32_t MOTOR_PWM_HZ   = 20000;
static const uint8_t  MOTOR_PWM_BITS = 10;     // 0..1023
static const uint8_t  MOTOR_CH0      = 0;      // canales LEDC 0..3 (motores)

// ---------- Servos y ESC (50 Hz) ----------
static const uint32_t SERVO_HZ   = 50;
static const uint8_t  SERVO_BITS = 16;
static const uint8_t  SERVO_CH0  = 4;          // canales LEDC 4..7: ESC1, ESC2, SERVO_L, SERVO_R
static const int SERVO_MIN_US = 1000, SERVO_MID_US = 1500, SERVO_MAX_US = 2000;
static const int ESC_OFF_US = 1000;
static const int ESC_TEST_LIMIT_US = 1300;     // tope del acelerador en la prueba 7
static const int SERVO_SPAN = 450;             // us de recorrido con la palanca al tope
static const float SERVO_DIFF = 0.5f;          // CH1 mueve los servos opuestos a la mitad

// ---------- iBUS y control por radio ----------
static const uint32_t IBUS_BAUD       = 115200;
static const uint32_t FAILSAFE_MS     = 100;   // sin tramas validas -> freno y desarme
static const int DEADBAND_US          = 80;
static const int CENTER_TOL_US        = 60;
static const uint32_t ARM_MS          = 1000;
static const int EDF_LOCK_US          = 1050;  // VrA y VrB por debajo de esto para habilitar las EDF
static const float EDF_RAMP_PER_S     = 2.0f;  // recorrido completo por segundo
static const float TRAC_RAMP_PER_S    = 2.5f;

}  // namespace cfg
