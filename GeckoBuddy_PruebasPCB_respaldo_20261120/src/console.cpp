#include "console.h"
#include "actuators.h"

namespace con {

int key() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c == '\r' || c == '\n') continue;
    return c;
  }
  return -1;
}

bool stopPressed(int k) { return k == 'x' || k == 'X' || k == ' '; }

void flushInput() {
  delay(20);
  while (Serial.available() > 0) Serial.read();
}

String readLine(uint32_t timeoutMs) {
  String s;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r' || c == '\n') {
        if (s.length() > 0) { s.trim(); return s; }
        continue;
      }
      s += c;
    }
    delay(5);
  }
  s.trim();
  return s;
}

bool confirm(const __FlashStringHelper* question) {
  flushInput();
  Serial.println(question);
  Serial.println(F("Escribe SI y Enter para seguir (cualquier otra cosa cancela):"));
  String s = readLine();
  s.toUpperCase();
  bool ok = (s == "SI" || s == "S\xC3\x8D" || s == "S\xC3\xAD");   // SI, SÍ o Sí
  Serial.println(ok ? F("-> Confirmado.") : F("-> Cancelado."));
  return ok;
}

void line(char c, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) Serial.print(c);
  Serial.println();
}

bool waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    int k = key();
    if (stopPressed(k)) {
      act::allStop();
      Serial.println(F("\n*** PARO: STBY bajo, motores frenados, ESC a 1000 us ***"));
      return false;
    }
    delay(5);
  }
  return true;
}

}  // namespace con
