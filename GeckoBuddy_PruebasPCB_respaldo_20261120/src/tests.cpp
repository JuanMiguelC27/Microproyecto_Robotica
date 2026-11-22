#include "tests.h"
#include "config.h"
#include "actuators.h"
#include "battery.h"
#include "console.h"

namespace tests {

static IBus* ibus = nullptr;
void setIBus(IBus* ib) { ibus = ib; }

// ------------------------------------------------------------------ utilidades
static void header(const __FlashStringHelper* title) {
  Serial.println();
  con::line('=');
  Serial.println(title);
  con::line('=');
}

static bool exitKey(int k) {           // 'x' / espacio = paro, 'q' = salir
  if (con::stopPressed(k)) {
    act::allStop();
    Serial.println(F("\n*** PARO: STBY bajo, motores frenados, ESC a 1000 us ***"));
    return true;
  }
  return k == 'q' || k == 'Q';
}

static float motorCap() {              // tope de PWM para que el motor de 6 V no pase de 6 V medios
  bat::Reading r = bat::read(bat::TRAC, 8);
  if (!r.present || r.volts <= cfg::V_MOTOR) return 1.0f;
  return cfg::V_MOTOR / r.volts;
}

static void printBattery(bat::Id id) {
  bat::Reading r = bat::read(id);
  Serial.printf("  %-26s pin %4lu mV  ->  %5.2f V  %s%s\n", bat::name(id), (unsigned long)r.pin_mV, r.volts, r.state,
                r.pin_mV > 2450 ? "  (pin sobre 2,45 V: fuera del rango lineal)" : "");
}

// ------------------------------------------------------------------ 1. ESP32 y D12
void t1_esp32() {
  header(F("1. ESP32 y puente D12 a GND"));
  Serial.printf("Chip: %s rev %d, %d nucleos a %lu MHz\n", ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
                (unsigned long)ESP.getCpuFreqMHz());
  Serial.printf("Flash: %lu MB   RAM libre: %lu bytes\n", (unsigned long)(ESP.getFlashChipSize() / (1024UL * 1024UL)),
                (unsigned long)ESP.getFreeHeap());
  uint64_t mac = ESP.getEfuseMac();
  Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", (uint8_t)(mac), (uint8_t)(mac >> 8), (uint8_t)(mac >> 16),
                (uint8_t)(mac >> 24), (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));

  // Con el pull-up interno activado (unos 45 k), un pin unido a GND sigue leyendo 0.
  // Si el puente no esta, el pull-up lo sube a 1. Es seguro: solo circulan ~70 uA.
  pinMode(cfg::PIN_GND_BRIDGE, INPUT_PULLUP);
  delay(5);
  int highs = 0;
  for (int i = 0; i < 50; i++) { highs += digitalRead(cfg::PIN_GND_BRIDGE); delay(1); }
  pinMode(cfg::PIN_GND_BRIDGE, INPUT);
  Serial.println();
  if (highs == 0) Serial.println(F("D12 = GND OK: el puente de la PCB esta bien (50 de 50 lecturas en 0)."));
  else Serial.printf("FALLA: D12 leyo 1 en %d de 50 lecturas. El puente D12-GND no esta hecho o tiene mala soldadura.\n", highs);
  Serial.println(F("Si la ESP32 arranco y ves este mensaje, D12 en bajo al encender es correcto (flash a 3,3 V)."));

  Serial.println(F("\nMapa de pines que usan las pruebas:"));
  for (uint8_t m = 0; m < 4; m++)
    Serial.printf("  %-34s PWM D%-2u IN1 D%-2u IN2 D%-2u %s\n", cfg::MOTORS[m].name, cfg::MOTORS[m].pwm, cfg::MOTORS[m].in1,
                  cfg::MOTORS[m].in2, cfg::MOTORS[m].invert ? "(espejo)" : "");
  Serial.printf("  STBY D%u, iBUS D%u, VBAT control D%u / traccion D%u / 3S D%u\n", cfg::PIN_STBY, cfg::PIN_IBUS_RX,
                cfg::PIN_VBAT_CTRL, cfg::PIN_VBAT_TRAC, cfg::PIN_VBAT_PROP);
  Serial.printf("  ESC1 D%u, ESC2 D%u, servo izq. D%u, servo der. D%u, D12 = GND (solo entrada)\n", cfg::PIN_ESC1, cfg::PIN_ESC2,
                cfg::PIN_SERVO_L, cfg::PIN_SERVO_R);
  Serial.println(F("Nota: el LED azul de la tarjeta esta en D2 (M4_IN2): se prende cuando trabaja el motor 4."));
}

// ------------------------------------------------------------------ 2. Baterias
static void calibrate() {
  Serial.println(F("Calibracion: 1 = 2S control, 2 = 2S traccion, 3 = 3S. Escribe el numero y Enter:"));
  String s = con::readLine(30000);
  int n = s.toInt();
  if (n < 1 || n > 3) { Serial.println(F("Cancelado.")); return; }
  bat::Id id = (bat::Id)(n - 1);
  Serial.println(F("Mide esa bateria con el multimetro (en sus terminales) y escribe el valor en voltios, por ejemplo 8.31:"));
  float real = con::readLine(60000).toFloat();
  bat::Reading r = bat::read(id, 64);
  float sinCal = r.pin_mV / 1000.0f * bat::divider(id);
  if (real < 1.0f || sinCal < 0.5f) { Serial.println(F("Valores no validos. Cancelado.")); return; }
  float k = real / sinCal;
  const char* nombre = id == bat::CTRL ? "CAL_CTRL" : id == bat::TRAC ? "CAL_TRAC" : "CAL_PROP";
  Serial.printf("Lectura sin calibrar %.3f V, multimetro %.3f V -> factor %.4f\n", sinCal, real, k);
  Serial.printf("Copia esta linea en include/config.h y vuelve a cargar el programa:\n  static const float %s = %.4ff;\n", nombre, k);
  if (k < 0.93f || k > 1.07f) Serial.println(F("Ojo: el error pasa del 7 %. Revisa los valores de las resistencias del divisor."));
}

void t2_baterias() {
  header(F("2. Baterias y divisores"));
  Serial.println(F("Conectado: 2S de control en J2, 2S de traccion en J6 (S2 encendido) y, si ya esta, la 3S por el XT60 J7 de la PCB-C y W1."));
  Serial.println(F("Compara cada valor con el multimetro. 'c' = calibrar, 'x' o 'q' = salir.\n"));
  con::flushInput();
  while (true) {
    for (uint8_t i = 0; i < 3; i++) printBattery((bat::Id)i);
    float cap = motorCap();
    Serial.printf("  Tope de PWM de los motores: %d %%\n\n", (int)(cap * 100 + 0.5f));
    uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
      int k = con::key();
      if (exitKey(k)) return;
      if (k == 'c' || k == 'C') { calibrate(); con::flushInput(); }
      delay(10);
    }
  }
}

// ------------------------------------------------------------------ 3. STBY y LED
void t3_stby_led() {
  header(F("3. STBY y LED de estado"));
  act::allMotorsCoast();
  Serial.println(F("'s' = cambiar STBY, 'p' = parpadeo 10 s, 'x' o 'q' = salir."));
  Serial.println(F("Con STBY alto: LED encendido y ~3,3 V en el pin STBY de cada TB6612 (mide contra GND)."));
  Serial.println(F("Con STBY bajo: LED apagado y 0 V. Si el LED no prende nunca, revisa su polaridad (anodo hacia STBY)."));
  con::flushInput();
  act::setStby(false);
  Serial.println(F("STBY = BAJO (LED apagado)"));
  while (true) {
    int k = con::key();
    if (exitKey(k)) { act::setStby(false); return; }
    if (k == 's' || k == 'S') {
      act::setStby(!act::stby());
      Serial.println(act::stby() ? F("STBY = ALTO (LED encendido)") : F("STBY = BAJO (LED apagado)"));
    }
    if (k == 'p' || k == 'P') {
      Serial.println(F("Parpadeo 1 Hz durante 10 s..."));
      for (int i = 0; i < 20; i++) {
        act::setStby(i % 2 == 0);
        if (!con::waitMs(500)) return;
      }
      act::setStby(false);
      Serial.println(F("Fin del parpadeo. STBY = BAJO"));
    }
    delay(10);
  }
}

// ------------------------------------------------------------------ 4. Motores
static bool rampTo(uint8_t m, float target, uint32_t ms) {     // cambia el mando de a poco
  float start = act::motorCmd(m);
  const int steps = 15;
  for (int i = 1; i <= steps; i++) {
    act::motorDrive(m, start + (target - start) * i / steps);
    if (!con::waitMs(ms / steps)) return false;
  }
  return true;
}

static bool step(uint8_t m, const char* text, float frac, int mode, uint32_t holdMs) {
  // mode: 0 = andar con frac (signo = sentido del robot), 1 = freno, 2 = rueda libre
  float cap = motorCap();
  bat::Reading r = bat::read(bat::TRAC, 8);
  Serial.printf("  %-44s", text);
  if (mode == 0) {
    float target = frac * cap;
    Serial.printf(" PWM %3d %%  VBAT %.2f V\n", (int)(fabsf(target) * 100 + 0.5f), r.volts);
    if (!rampTo(m, target, 300)) return false;
  } else if (mode == 1) {
    act::motorBrake(m);
    Serial.printf(" IN1=IN2=1   VBAT %.2f V\n", r.volts);
  } else {
    act::motorCoast(m);
    Serial.printf(" IN1=IN2=0   VBAT %.2f V\n", r.volts);
  }
  return con::waitMs(holdMs);
}

static bool sequence(uint8_t m) {
  Serial.printf("\n>> %s\n", cfg::MOTORS[m].name);
  act::motorBrake(m);
  act::setStby(true);
  bool ok = step(m, "Freno con STBY alto: la rueda no gira", 0, 1, 1500) &&
            step(m, "Adelante 30 %: gira despacio hacia ADELANTE", 0.30f, 0, 2000) &&
            step(m, "Adelante 60 %", 0.60f, 0, 2000) &&
            step(m, "Adelante al tope (6 V medios)", 1.00f, 0, 2000) &&
            step(m, "Freno corto: debe parar casi en seco", 0, 1, 1500) &&
            step(m, "Atras 30 %: gira hacia ATRAS", -0.30f, 0, 2000) &&
            step(m, "Atras al tope", -1.00f, 0, 2000) &&
            step(m, "Rueda libre: se detiene despacio", 0, 2, 2000);
  act::motorBrake(m);
  act::setStby(false);
  if (ok) Serial.println(F("  Listo. Si giro al reves, intercambia sus dos cables o cambia 'invert' en config.h."));
  return ok;
}

static void manual() {
  uint8_t m = 0;
  float cmd = 0;
  Serial.println(F("Modo manual: 1-4 = motor, 'w' = +10 %, 's' = -10 %, 'b' = freno, 'c' = libre, 'x' = paro y salir."));
  act::setStby(true);
  act::motorBrake(m);
  while (true) {
    int k = con::key();
    if (exitKey(k)) { act::allStop(); return; }
    bool changed = true;
    if (k >= '1' && k <= '4') { act::motorBrake(m); m = k - '1'; cmd = 0; act::motorBrake(m); }
    else if (k == 'w' || k == 'W') { cmd = min(1.0f, cmd + 0.1f); act::motorDrive(m, cmd * motorCap()); }
    else if (k == 's' || k == 'S') { cmd = max(-1.0f, cmd - 0.1f); act::motorDrive(m, cmd * motorCap()); }
    else if (k == 'b' || k == 'B') { cmd = 0; act::motorBrake(m); }
    else if (k == 'c' || k == 'C') { cmd = 0; act::motorCoast(m); }
    else changed = false;
    if (changed)
      Serial.printf("  %s  mando %+4d %%  (PWM real %d %%, tope %d %%)\n", cfg::MOTORS[m].name, (int)(cmd * 100),
                    (int)(fabsf(cmd) * motorCap() * 100 + 0.5f), (int)(motorCap() * 100 + 0.5f));
    delay(10);
  }
}

void t4_motores() {
  header(F("4. Drivers TB6612 y motores"));
  bat::Reading r = bat::read(bat::TRAC);
  Serial.printf("2S de traccion: %.2f V (%s). Tope de PWM: %d %%\n", r.volts, r.state, (int)(motorCap() * 100 + 0.5f));
  if (!r.present) Serial.println(F("AVISO: no hay tension en J6. Sin VM los motores no giran (revisa F2, S2 y W7)."));
  if (!con::confirm(F("Las 4 ruedas deben estar EN EL AIRE (robot sobre un soporte). Listo?"))) return;
  con::flushInput();
  while (true) {
    Serial.println(F("\n1-4 = secuencia de un motor, 't' = los cuatro seguidos, 'm' = manual, 'q' = volver al menu."));
    int k = -1;
    while (k < 0) { k = con::key(); delay(10); }
    if (exitKey(k)) return;
    if (k >= '1' && k <= '4') { if (!sequence(k - '1')) return; }
    else if (k == 't' || k == 'T') { for (uint8_t m = 0; m < 4; m++) if (!sequence(m)) return; }
    else if (k == 'm' || k == 'M') { manual(); return; }
  }
}

// ------------------------------------------------------------------ 5. iBUS
void t5_ibus() {
  header(F("5. Receptor FS-iA6B por i-BUS (D34)"));
  Serial.println(F("Control FS-i6X encendido, RX Setup -> Output mode -> Serial i-BUS, cable en el puerto i-BUS SERVO."));
  Serial.println(F("Mueve las palancas: cada canal debe ir de 1000 a 2000 (centro ~1500). 'x' o 'q' = salir.\n"));
  con::flushInput();
  uint32_t t0 = millis(), lastPrint = 0, okPrev = ibus->framesOk(), lastRate = millis();
  float rate = 0;
  bool warned = false;
  while (true) {
    ibus->update();
    int k = con::key();
    if (exitKey(k)) return;
    uint32_t now = millis();
    if (now - lastRate >= 1000) {
      rate = (ibus->framesOk() - okPrev) * 1000.0f / (now - lastRate);
      okPrev = ibus->framesOk();
      lastRate = now;
    }
    if (now - lastPrint >= 250) {
      lastPrint = now;
      if (ibus->fresh(cfg::FAILSAFE_MS)) {
        Serial.printf("CH1 %4u CH2 %4u CH3 %4u CH4 %4u CH5 %4u CH6 %4u | %5.1f tramas/s, buenas %lu, malas %lu\n", ibus->ch(0),
                      ibus->ch(1), ibus->ch(2), ibus->ch(3), ibus->ch(4), ibus->ch(5), rate, (unsigned long)ibus->framesOk(),
                      (unsigned long)ibus->framesBad());
        warned = false;
      } else if (now - t0 > 2000 && !warned) {
        warned = true;
        if (ibus->bytesIn() == 0) {
          Serial.println(F("No llega NADA a D34. Revisa: GND comun receptor-ESP32 (J1-2), cable de senal en J1-3,"));
          Serial.println(F("puerto i-BUS SERVO (no SENS), receptor alimentado (LED) y enlazado con el control."));
        } else if (ibus->framesOk() == 0) {
          Serial.println(F("Llegan bytes pero ninguna trama valida. El control no esta en Serial i-BUS,"));
          Serial.println(F("o el cable esta en el puerto SENS o en un canal PWM."));
        } else {
          Serial.println(F("Se perdio la senal (mas de 100 ms sin tramas). Es lo que activa el failsafe del robot."));
        }
      }
    }
    delay(2);
  }
}

// ------------------------------------------------------------------ 6. Servos
void t6_servos() {
  header(F("6. Servos (PCB-C J5 y J6)"));
  Serial.println(F("Los servos se alimentan por +5V_SERVO: BEC del ESC1 (3S conectada y S3 encendido)"));
  Serial.println(F("o 5 V provisionales en J3-2 de la PCB-C. Los ESC quedan en 1000 us durante esta prueba."));
  if (!con::confirm(F("Los servos pueden moverse libremente (sin piezas que choquen)?"))) return;
  act::writeUs(act::ESC1, cfg::ESC_OFF_US);
  act::writeUs(act::ESC2, cfg::ESC_OFF_US);
  uint8_t sel = 2;   // 2 = izquierdo, 3 = derecho, 4 = ambos
  Serial.println(F("'c' = centro, 'b' = barrido, 'l'/'r'/'a' = izquierdo/derecho/ambos, '+'/'-' = 50 us, 'x' o 'q' = salir."));
  Serial.println(F("'m' = modo medicion: 3,3 V fijos en la senal de los dos servos para seguirla con el multimetro."));
  Serial.println(F("Por defecto se mueve solo el izquierdo: pulsa 'a' para mover los dos."));
  bool medir = false;
  Serial.println(F("Si un servo zumba fuerte en un extremo, esta forzando: vuelvelo al centro."));
  act::writeUs(act::SERVO_L, cfg::SERVO_MID_US);
  act::writeUs(act::SERVO_R, cfg::SERVO_MID_US);
  con::flushInput();
  while (true) {
    int k = con::key();
    if (exitKey(k)) return;
    bool show = true;
    auto apply = [&](int delta) {
      for (uint8_t w = 2; w <= 3; w++)
        if (sel == 4 || sel == w) act::writeUs(w, act::readUs(w) + delta);
    };
    if (k == 'm' || k == 'M') {
      medir = !medir;
      if (medir) {
        act::writeLevel(act::SERVO_L, true);
        act::writeLevel(act::SERVO_R, true);
        Serial.println(F("MODO MEDICION: desconecta los servos. Senal fija en 3,3 V. Mide contra GND (debe dar ~3,3 V):"));
        Serial.println(F("  1) pin D19 y D18 de la ESP32   2) J4-1 y J4-2 de la placa principal"));
        Serial.println(F("  3) J1-1 y J1-2 de la PCB-C     4) pin 1 (S) de J5 y de J6 de la PCB-C"));
        Serial.println(F("Donde deje de dar 3,3 V esta el corte. 'm' otra vez para volver a 1500 us."));
      } else {
        act::writeUs(act::SERVO_L, cfg::SERVO_MID_US);
        act::writeUs(act::SERVO_R, cfg::SERVO_MID_US);
        Serial.println(F("Modo medicion apagado: servos en 1500 us."));
      }
      show = false;
    }
    else if (medir) { show = false; }
    else if (k == 'c' || k == 'C') { act::writeUs(act::SERVO_L, cfg::SERVO_MID_US); act::writeUs(act::SERVO_R, cfg::SERVO_MID_US); }
    else if (k == 'l' || k == 'L') sel = 2;
    else if (k == 'r' || k == 'R') sel = 3;
    else if (k == 'a' || k == 'A') sel = 4;
    else if (k == '+') apply(50);
    else if (k == '-') apply(-50);
    else if (k == 'b' || k == 'B') {
      Serial.println(F("Barrido 1500 -> 1000 -> 2000 -> 1500 ..."));
      int seq[] = {1500, 1000, 2000, 1500};
      for (int i = 0; i < 3; i++) {
        int a = seq[i], b = seq[i + 1], s = (b > a) ? 10 : -10;
        for (int us = a; us != b; us += s) {
          for (uint8_t w = 2; w <= 3; w++)
            if (sel == 4 || sel == w) act::writeUs(w, us);
          if (!con::waitMs(20)) return;
        }
      }
    } else show = false;
    if (show)
      Serial.printf("  seleccion: %s | izquierdo %d us, derecho %d us\n", sel == 2 ? "izquierdo" : sel == 3 ? "derecho" : "ambos",
                    act::readUs(act::SERVO_L), act::readUs(act::SERVO_R));
    delay(10);
  }
}

// ------------------------------------------------------------------ 7. ESC y EDF
void t7_esc() {
  header(F("7. ESC y EDF"));
  Serial.println(F("PELIGRO: la EDF gira a miles de rpm. Robot sujeto, EDF fija, nada suelto cerca de la entrada,"));
  Serial.println(F("manos y cables lejos. 3S conectada con S3. Tope de esta prueba: ") );
  Serial.printf("%d us (cambialo en config.h, ESC_TEST_LIMIT_US).\n", cfg::ESC_TEST_LIMIT_US);
  if (!con::confirm(F("Todo asegurado?"))) return;
  Serial.println(F("Que ESC: 1 = ESC1 (EDF izq.), 2 = ESC2 (EDF der.), 3 = ambos. Numero y Enter:"));
  int n = con::readLine(30000).toInt();
  if (n < 1 || n > 3) { Serial.println(F("Cancelado.")); return; }
  bool e1 = (n == 1 || n == 3), e2 = (n == 2 || n == 3);
  act::writeUs(act::ESC1, cfg::ESC_OFF_US);
  act::writeUs(act::ESC2, cfg::ESC_OFF_US);
  Serial.println(F("Armando: 1000 us durante 3 s (los ESC pitan)..."));
  if (!con::waitMs(3000)) return;
  Serial.println(F("'+'/'-' = 25 us, '0' = apagar, 'x' o espacio = PARO. Sin teclas por 10 s vuelve sola a 1000 us."));
  con::flushInput();
  int us = cfg::ESC_OFF_US;
  uint32_t lastKey = millis(), lastPrint = 0;
  while (true) {
    int k = con::key();
    if (exitKey(k)) { act::allStop(); return; }
    if (k == '+') { us = min(cfg::ESC_TEST_LIMIT_US, us + 25); lastKey = millis(); }
    else if (k == '-') { us = max(cfg::ESC_OFF_US, us - 25); lastKey = millis(); }
    else if (k == '0') { us = cfg::ESC_OFF_US; lastKey = millis(); }
    if (us > cfg::ESC_OFF_US && millis() - lastKey > 10000) {
      us = cfg::ESC_OFF_US;
      Serial.println(F("10 s sin teclas: acelerador a 1000 us."));
    }
    if (e1) act::writeUs(act::ESC1, us);
    if (e2) act::writeUs(act::ESC2, us);
    if (millis() - lastPrint >= 400 || k == '+' || k == '-' || k == '0') {
      lastPrint = millis();
      bat::Reading r = bat::read(bat::PROP, 8);
      Serial.printf("  acelerador %d us (%d %%)   3S %.2f V %s\n", us, (us - 1000) / 10, r.volts, r.state);
    }
    delay(10);
  }
}

// ------------------------------------------------------------------ 8. Integracion
static float stick(int us) {
  int d = us - 1500;
  if (abs(d) < cfg::DEADBAND_US) return 0;
  d += d > 0 ? -cfg::DEADBAND_US : cfg::DEADBAND_US;
  float v = d / float(500 - cfg::DEADBAND_US);
  return v > 1 ? 1 : v < -1 ? -1 : v;
}

void t8_integracion() {
  header(F("8. Integracion: todo con el control"));
  Serial.println(F("Robot sujeto, ruedas en el aire, EDF fijas. Es la misma logica del firmware del robot:"));
  Serial.println(F("- arma con CH3 (avance) y CH4 (giro) al centro 1 s; sin senal 100 ms -> freno, STBY bajo y EDF a 1000 us"));
  Serial.println(F("- EDF bloqueadas hasta bajar VrA (CH5) y VrB (CH6) a la vez; luego suben con rampa"));
  Serial.println(F("- CH2 inclina los servos juntos, CH1 los mueve opuestos"));
  if (!con::confirm(F("Todo asegurado?"))) return;
  act::allStop();
  con::flushInput();
  bool armed = false, edf = false;
  uint32_t centeredSince = 0, lastStep = millis(), lastPrint = 0;
  float vTrac = bat::read(bat::TRAC).volts, vCtrl = bat::read(bat::CTRL).volts, vProp = bat::read(bat::PROP).volts;
  float esc[2] = {1000, 1000}, wheel[4] = {0, 0, 0, 0};
  uint8_t iLeft = 0, iRight = 0;   // primer motor de cada lado, para el reporte
  for (int8_t m = 3; m >= 0; m--) {
    if (cfg::MOTORS[m].right) iRight = m; else iLeft = m;
  }
  while (true) {
    ibus->update();
    int k = con::key();
    if (exitKey(k)) { act::allStop(); return; }
    uint32_t now = millis();
    if (now - lastStep < 10) { delay(1); continue; }
    float dt = (now - lastStep) / 1000.0f;
    lastStep = now;

    vTrac += 0.1f * (bat::read(bat::TRAC, 4).volts - vTrac);
    vCtrl += 0.1f * (bat::read(bat::CTRL, 4).volts - vCtrl);
    vProp += 0.1f * (bat::read(bat::PROP, 4).volts - vProp);
    float cap = vTrac > cfg::V_MOTOR ? cfg::V_MOTOR / vTrac : 1.0f;

    bool link = ibus->fresh(cfg::FAILSAFE_MS);
    if (!link) {
      if (armed) Serial.println(F(">> SIN SENAL: freno, STBY bajo, EDF a 1000 us"));
      armed = false;
      centeredSince = 0;
      for (uint8_t m = 0; m < 4; m++) wheel[m] = 0;
      act::allStop();
      esc[0] = esc[1] = cfg::ESC_OFF_US;
    } else {
      int ch[6];
      for (uint8_t i = 0; i < 6; i++) ch[i] = ibus->ch(i);
      bool centered = abs(ch[2] - 1500) < cfg::CENTER_TOL_US && abs(ch[3] - 1500) < cfg::CENTER_TOL_US;
      if (!armed) {
        if (centered) {
          if (!centeredSince) centeredSince = now ? now : 1;
          if (now - centeredSince >= cfg::ARM_MS) {
            armed = true;
            act::setStby(true);
            Serial.println(F(">> ARMADO: STBY alto"));
          }
        } else centeredSince = 0;
      }
      // ruedas: mezcla tanque con rampa
      float thr = stick(ch[2]), turn = stick(ch[3]);
      float left = constrain(thr + turn, -1.0f, 1.0f), right = constrain(thr - turn, -1.0f, 1.0f);
      float target[4];
      for (uint8_t m = 0; m < 4; m++) target[m] = cfg::MOTORS[m].right ? right : left;
      float maxStep = cfg::TRAC_RAMP_PER_S * dt;
      for (uint8_t m = 0; m < 4; m++) {
        float d = constrain(target[m] - wheel[m], -maxStep, maxStep);
        wheel[m] = armed ? wheel[m] + d : 0;
        if (armed) act::motorDrive(m, wheel[m] * cap); else act::motorBrake(m);
      }
      // EDF
      if (!edf && ch[4] <= cfg::EDF_LOCK_US && ch[5] <= cfg::EDF_LOCK_US) {
        edf = true;
        Serial.println(F(">> EDF habilitadas"));
      }
      float inc = cfg::EDF_RAMP_PER_S * 1000.0f * dt;
      for (uint8_t i = 0; i < 2; i++) {
        float tgt = edf ? ch[4 + i] : cfg::ESC_OFF_US;
        esc[i] = esc[i] < tgt ? min(tgt, esc[i] + inc) : max(tgt, esc[i] - inc);
      }
      // servos
      float b = (ch[1] - 1500) / 500.0f * cfg::SERVO_SPAN, dd = (ch[0] - 1500) / 500.0f * cfg::SERVO_SPAN * cfg::SERVO_DIFF;
      act::writeUs(act::SERVO_L, (int)(1500 + b + dd));
      act::writeUs(act::SERVO_R, (int)(1500 - b + dd));
    }
    act::writeUs(act::ESC1, (int)esc[0]);
    act::writeUs(act::ESC2, (int)esc[1]);

    if (now - lastPrint >= 500) {
      lastPrint = now;
      Serial.printf("%s %s | CH %4u %4u %4u %4u %4u %4u | rueda izq %+4d%% der %+4d%% (tope %d%%) | ESC %4d/%4d | servos %4d/%4d | "
                    "BT1 %.2f 2S %.2f 3S %.2f%s%s%s\n",
                    armed ? "ARM" : "---", link ? "OK " : "SIN", ibus->ch(0), ibus->ch(1), ibus->ch(2), ibus->ch(3), ibus->ch(4),
                    ibus->ch(5), (int)(wheel[iLeft] * cap * 100), (int)(wheel[iRight] * cap * 100), (int)(cap * 100), (int)esc[0], (int)esc[1],
                    act::readUs(act::SERVO_L), act::readUs(act::SERVO_R), vCtrl, vTrac, vProp,
                    vTrac < cfg::V2S_LOW ? "  TRACCION BAJA" : "", vCtrl < cfg::V2S_LOW ? "  BT1 BAJA" : "",
                    vProp < cfg::V3S_CRIT ? "  3S CRITICA" : vProp < cfg::V3S_LOW ? "  3S BAJA" : "");
    }
  }
}

// ------------------------------------------------------------------ 9. Estado
void t9_estado() {
  header(F("9. Estado de todo"));
  pinMode(cfg::PIN_GND_BRIDGE, INPUT_PULLUP);
  delay(2);
  int d12 = digitalRead(cfg::PIN_GND_BRIDGE);
  pinMode(cfg::PIN_GND_BRIDGE, INPUT);
  Serial.printf("D12 (puente a GND): %s\n", d12 == 0 ? "OK, en GND" : "FALLA, no esta en GND");
  Serial.printf("STBY: %s\n", act::stby() ? "ALTO" : "BAJO");
  for (uint8_t i = 0; i < 3; i++) printBattery((bat::Id)i);
  Serial.printf("Tope de PWM: %d %%\n", (int)(motorCap() * 100 + 0.5f));
  for (uint8_t m = 0; m < 4; m++) {
    act::MotorMode md = act::motorMode(m);
    Serial.printf("  %-34s %s\n", cfg::MOTORS[m].name,
                  md == act::MotorMode::Brake ? "freno" : md == act::MotorMode::Coast ? "libre" : "andando");
  }
  for (uint8_t i = 0; i < 4; i++) Serial.printf("  %-16s %d us\n", act::out50Name(i), act::readUs(i));
  ibus->update();
  Serial.printf("iBUS: %s, tramas buenas %lu, malas %lu, bytes %lu\n", ibus->fresh(cfg::FAILSAFE_MS) ? "con senal" : "sin senal",
                (unsigned long)ibus->framesOk(), (unsigned long)ibus->framesBad(), (unsigned long)ibus->bytesIn());
}

}  // namespace tests
