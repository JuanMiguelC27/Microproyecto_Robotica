// Pruebas paso a paso. Cada una vuelve al menu con 'x' (paro) o 'q'.
#pragma once
#include "ibus.h"

namespace tests {
void setIBus(IBus* ib);
void t1_esp32();
void t2_baterias();
void t3_stby_led();
void t4_motores();
void t5_ibus();
void t6_servos();
void t7_esc();
void t8_integracion();
void t9_estado();
}  // namespace tests
