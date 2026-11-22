// Motores (TB6612FNG), STBY, servos y ESC. Todas las salidas pasan por aqui.
#pragma once
#include <Arduino.h>

namespace act {

enum class MotorMode : uint8_t { Coast, Brake, Drive };

void begin();                         // todo en estado seguro: STBY bajo, motores libres, ESC 1000 us

// --- STBY ---
void setStby(bool on);
bool stby();

// --- Motores (m = 0..3) ---
// cmd -1..+1 en sentido del ROBOT (la inversion de M3/M4 se aplica sola). Ya viene con el tope aplicado.
void motorDrive(uint8_t m, float cmd);
void motorBrake(uint8_t m);           // IN1 = IN2 = alto: freno corto
void motorCoast(uint8_t m);           // IN1 = IN2 = bajo, PWM 0: rueda libre
void allMotorsBrake();
void allMotorsCoast();
float motorCmd(uint8_t m);
MotorMode motorMode(uint8_t m);

// --- Servos y ESC (us) ---
enum Out50 : uint8_t { ESC1 = 0, ESC2 = 1, SERVO_L = 2, SERVO_R = 3 };
void writeUs(uint8_t which, int us);
int  readUs(uint8_t which);
// Solo para medir con el multimetro: deja la salida fija en 3,3 V (true) o en 0 V (false).
void writeLevel(uint8_t which, bool high);
const char* out50Name(uint8_t which);

// --- Paro total: STBY bajo, freno, ESC a 1000 us (los servos se quedan donde estan) ---
void allStop();

}  // namespace act
