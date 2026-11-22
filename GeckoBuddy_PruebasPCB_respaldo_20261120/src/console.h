// Monitor serie: menu, teclas y paro de emergencia.
#pragma once
#include <Arduino.h>

namespace con {

int  key();                                   // -1 si no hay tecla; ignora fin de linea
bool stopPressed(int k);                      // 'x', 'X' o espacio
String readLine(uint32_t timeoutMs = 60000);  // espera una linea (Enter)
bool confirm(const __FlashStringHelper* question);   // el usuario debe escribir SI
void flushInput();
void line(char c = '-', uint8_t n = 72);
bool waitMs(uint32_t ms);                     // espera ms; devuelve false si piden paro

}  // namespace con
