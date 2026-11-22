#include "actuators.h"
#include "config.h"
#include "pwm_out.h"

namespace act {

static PwmOut motorPwm[4];
static PwmOut out50[4];
static float cmd_[4] = {0, 0, 0, 0};
static MotorMode mode_[4] = {MotorMode::Coast, MotorMode::Coast, MotorMode::Coast, MotorMode::Coast};
static int us_[4] = {cfg::ESC_OFF_US, cfg::ESC_OFF_US, cfg::SERVO_MID_US, cfg::SERVO_MID_US};
static bool stby_ = false;

void begin() {
  // 1) STBY bajo antes que nada: drivers apagados
  pinMode(cfg::PIN_STBY, OUTPUT);
  digitalWrite(cfg::PIN_STBY, LOW);
  stby_ = false;
  // 2) D12 esta unido a GND en la PCB: queda como entrada SIN pull-up (nunca salida)
  pinMode(cfg::PIN_GND_BRIDGE, INPUT);
  // 3) Motores libres
  for (uint8_t m = 0; m < 4; m++) {
    pinMode(cfg::MOTORS[m].in1, OUTPUT);
    pinMode(cfg::MOTORS[m].in2, OUTPUT);
    digitalWrite(cfg::MOTORS[m].in1, LOW);
    digitalWrite(cfg::MOTORS[m].in2, LOW);
    motorPwm[m].begin(cfg::MOTORS[m].pwm, cfg::MOTOR_CH0 + m, cfg::MOTOR_PWM_HZ, cfg::MOTOR_PWM_BITS);
    mode_[m] = MotorMode::Coast;
    cmd_[m] = 0;
  }
  // 4) ESC en 1000 us (apagado) y servos al centro
  const uint8_t pins[4] = {cfg::PIN_ESC1, cfg::PIN_ESC2, cfg::PIN_SERVO_L, cfg::PIN_SERVO_R};
  for (uint8_t i = 0; i < 4; i++) {
    out50[i].begin(pins[i], cfg::SERVO_CH0 + i, cfg::SERVO_HZ, cfg::SERVO_BITS);
    out50[i].writeMicros(us_[i]);
  }
}

void setStby(bool on) {
  digitalWrite(cfg::PIN_STBY, on ? HIGH : LOW);
  stby_ = on;
}
bool stby() { return stby_; }

void motorDrive(uint8_t m, float cmd) {
  if (m > 3) return;
  if (cmd > 1) cmd = 1;
  if (cmd < -1) cmd = -1;
  cmd_[m] = cmd;
  if (cmd > -0.005f && cmd < 0.005f) { motorBrake(m); return; }
  float c = cfg::MOTORS[m].invert ? -cmd : cmd;
  digitalWrite(cfg::MOTORS[m].in1, c > 0 ? HIGH : LOW);
  digitalWrite(cfg::MOTORS[m].in2, c > 0 ? LOW : HIGH);
  motorPwm[m].writeFraction(c > 0 ? c : -c);
  mode_[m] = MotorMode::Drive;
}

void motorBrake(uint8_t m) {
  if (m > 3) return;
  digitalWrite(cfg::MOTORS[m].in1, HIGH);
  digitalWrite(cfg::MOTORS[m].in2, HIGH);
  motorPwm[m].writeFraction(1.0f);
  mode_[m] = MotorMode::Brake;
  cmd_[m] = 0;
}

void motorCoast(uint8_t m) {
  if (m > 3) return;
  digitalWrite(cfg::MOTORS[m].in1, LOW);
  digitalWrite(cfg::MOTORS[m].in2, LOW);
  motorPwm[m].writeFraction(0.0f);
  mode_[m] = MotorMode::Coast;
  cmd_[m] = 0;
}

void allMotorsBrake() { for (uint8_t m = 0; m < 4; m++) motorBrake(m); }
void allMotorsCoast() { for (uint8_t m = 0; m < 4; m++) motorCoast(m); }
float motorCmd(uint8_t m) { return m < 4 ? cmd_[m] : 0; }
MotorMode motorMode(uint8_t m) { return m < 4 ? mode_[m] : MotorMode::Coast; }

void writeUs(uint8_t which, int us) {
  if (which > 3) return;
  if (us < cfg::SERVO_MIN_US) us = cfg::SERVO_MIN_US;
  if (us > cfg::SERVO_MAX_US) us = cfg::SERVO_MAX_US;
  us_[which] = us;
  out50[which].writeMicros(us);
}
int readUs(uint8_t which) { return which < 4 ? us_[which] : 0; }

void writeLevel(uint8_t which, bool high) {
  if (which > 3) return;
  out50[which].writeRaw(high ? out50[which].maxDuty() : 0);
}

const char* out50Name(uint8_t which) {
  static const char* n[4] = {"ESC1 (EDF izq.)", "ESC2 (EDF der.)", "servo izquierdo", "servo derecho"};
  return which < 4 ? n[which] : "?";
}

void allStop() {
  allMotorsBrake();
  setStby(false);
  writeUs(ESC1, cfg::ESC_OFF_US);
  writeUs(ESC2, cfg::ESC_OFF_US);
}

}  // namespace act
