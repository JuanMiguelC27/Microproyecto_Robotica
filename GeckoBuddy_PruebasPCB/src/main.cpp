// Gecko Buddy - programa de pruebas modulares de la placa principal (ESP32 DevKit V1) y la PCB-C.
// Monitor serie a 115200. Cada prueba corresponde a una etapa de montaje (ver LEEME.md).
// Paro inmediato en cualquier prueba: 'x' o barra espaciadora.
//
// Dos formas de usarlo (se elige el "entorno" en la barra de abajo de VS Code / PlatformIO):
//   menu_completo      -> menu con las 9 pruebas.
//   p1_esp32 ... p8_integracion -> un programa por seccion: arranca directo en esa prueba
//                         (SOLO_PRUEBA = numero). 'r' la repite, '9' muestra el estado.
#include <Arduino.h>
#include "config.h"
#include "actuators.h"
#include "battery.h"
#include "console.h"
#include "ibus.h"
#include "tests.h"

static IBus ibus;

[[maybe_unused]] static const char* testName(char k) {
  switch (k) {
    case '1': return "ESP32 y puente D12 a GND";
    case '2': return "Baterias y divisores";
    case '3': return "STBY y LED de estado";
    case '4': return "Drivers TB6612 y motores";
    case '5': return "Receptor i-BUS";
    case '6': return "Servos";
    case '7': return "ESC y EDF";
    case '8': return "Integracion con el control";
    case '9': return "Estado de todo";
    default:  return "";
  }
}

static bool runTest(char k) {
  switch (k) {
    case '1': tests::t1_esp32(); break;
    case '2': tests::t2_baterias(); break;
    case '3': tests::t3_stby_led(); break;
    case '4': tests::t4_motores(); break;
    case '5': tests::t5_ibus(); break;
    case '6': tests::t6_servos(); break;
    case '7': tests::t7_esc(); break;
    case '8': tests::t8_integracion(); break;
    case '9': tests::t9_estado(); break;
    default: return false;
  }
  return true;
}

#ifndef SOLO_PRUEBA
// ------------------------------------------------------------ menu completo
static void menu() {
  Serial.println();
  con::line('#');
  Serial.println(F("  GECKO BUDDY - PRUEBAS DE LA PCB (ESP32 DevKit V1)"));
  con::line('#');
  Serial.println(F("  1  ESP32 y puente D12 a GND          (etapa 1: solo USB)"));
  Serial.println(F("  2  Baterias y divisores              (etapa 2: J2 y J6; despues la 3S)"));
  Serial.println(F("  3  STBY y LED de estado              (etapas 1-2)"));
  Serial.println(F("  4  Drivers TB6612 y motores          (etapa 3: ruedas en el aire)"));
  Serial.println(F("  5  Receptor i-BUS                    (etapa 4)"));
  Serial.println(F("  6  Servos                            (etapa 5: PCB-C y +5V_SERVO)"));
  Serial.println(F("  7  ESC y EDF                         (etapa 6: robot sujeto)"));
  Serial.println(F("  8  Integracion con el control        (etapa 7)"));
  Serial.println(F("  9  Estado de todo"));
  Serial.println(F("  x  PARO (en cualquier momento)"));
  Serial.print(F("Elige una opcion: "));
}
#else
// ------------------------------------------------------------ un programa por seccion
#define STR2(x) #x
#define STR(x) STR2(x)
static const char SOLO = STR(SOLO_PRUEBA)[0];
static void menu() {
  Serial.println();
  con::line('#');
  Serial.print(F("  GECKO BUDDY - PRUEBA ")); Serial.print(SOLO); Serial.print(F(": ")); Serial.println(testName(SOLO));
  con::line('#');
  Serial.println(F("  r  repetir la prueba"));
  Serial.println(F("  9  estado de todo"));
  Serial.println(F("  x  PARO (en cualquier momento)"));
  Serial.print(F("Elige una opcion: "));
}
#endif

void setup() {
  act::begin();                 // lo primero: salidas en estado seguro (STBY bajo, ESC 1000 us)
  Serial.begin(115200);
  delay(300);
  bat::begin();
  ibus.begin(Serial2, cfg::PIN_IBUS_RX, cfg::IBUS_BAUD);
  tests::setIBus(&ibus);
  Serial.println(F("\nGecko Buddy listo. Salidas en estado seguro."));
#ifdef SOLO_PRUEBA
  Serial.println(F("Abre el monitor y presiona 'r' si no alcanzaste a ver el comienzo."));
  delay(1500);                  // tiempo para que el monitor serie se conecte
  runTest(SOLO);
  act::allStop();
  con::flushInput();
#endif
  menu();
}

void loop() {
  ibus.update();
  int k = con::key();
  if (k < 0) { delay(5); return; }
  Serial.println((char)k);
  if (k == 'x' || k == 'X' || k == ' ') {
    act::allStop();
    Serial.println(F("PARO: todo en estado seguro."));
  } else {
#ifdef SOLO_PRUEBA
    if (k == 'r' || k == 'R') k = SOLO;
    if (k != SOLO && k != '9') { Serial.println(F("Opcion no valida.")); k = 0; }
#endif
    if (k && !runTest((char)k)) Serial.println(F("Opcion no valida."));
  }
  act::allStop();               // al volver de cualquier prueba, todo queda seguro
  con::flushInput();
  menu();
}
