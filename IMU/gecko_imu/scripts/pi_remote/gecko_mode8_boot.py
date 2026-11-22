#!/usr/bin/env python3
"""Selecciona el modo 8 una vez por arranque mediante el puente serial."""

import os
import re
import sys
import time
from pathlib import Path


DIRECTORIO = Path(os.environ.get("GECKO_SERIAL_DIR", Path.home() / "gecko_serial"))
LOG = DIRECTORIO / "esp32.log"
FIFO = DIRECTORIO / "cmd.fifo"
RUNTIME = Path(os.environ["XDG_RUNTIME_DIR"])
# Ventana máxima, contada desde el arranque del sistema (no desde el inicio del
# servicio), para que reinicios del servicio no la amplíen.
VENTANA_S = float(os.environ.get("GECKO_VENTANA_S", "180"))
# Solo para pruebas: instante CLOCK_BOOTTIME que se toma como "arranque".
ORIGEN_BOOT = float(os.environ.get("GECKO_ORIGEN_BOOT", "0"))
SELLO = RUNTIME / "gecko-modo-8"
PARO = RUNTIME / "gecko-paro-manual"
MENU = b"Elige una opcion:"
CONFIRMACION = b"Escribe SI y Enter"
# Solo la 2S de control es obligatoria; la lectura de tracción a veces cae a 0 V por un fallo de medición.
CONTROL = re.compile(rb"2S control[^\r\n]*?->\s+(\d+\.\d+) V\s+([^\r\n]+)")


def desde_arranque():
    return time.clock_gettime(time.CLOCK_BOOTTIME) - ORIGEN_BOOT


def control_presente(lectura):
    """True si la última línea '2S control' de la prueba 9 indica batería conectada."""
    lineas = CONTROL.findall(lectura)
    return bool(lineas) and lineas[-1][1].strip() != b"NO CONECTADA"


def intercambiar(comando, esperar, segundos):
    """Envía una orden y espera solamente la respuesta posterior a su envío."""
    with LOG.open("rb") as registro:
        registro.seek(0, os.SEEK_END)
        fd = os.open(FIFO, os.O_WRONLY | os.O_NONBLOCK)
        try:
            os.write(fd, comando)
        finally:
            os.close(fd)
        fin = time.monotonic() + segundos
        respuesta = b""
        while time.monotonic() < fin:
            respuesta += registro.read()
            if esperar in respuesta:
                return respuesta
            time.sleep(0.1)
    raise TimeoutError(f"Sin respuesta a {comando!r}")


def main():
    if SELLO.exists() or PARO.exists():
        print("El arranque automático ya se procesó o se solicitó el paro manual.")
        return

    # El USB y la alimentación pueden tardar en estar disponibles al encender.
    en_menu = False
    while desde_arranque() < VENTANA_S:
        if PARO.exists():
            return
        try:
            if not en_menu:
                intercambiar(b"x", MENU, 1.5)
                en_menu = True
            lectura = intercambiar(b"9", MENU, 1.5)
            if b"9. Estado de todo" in lectura and control_presente(lectura):
                break
        except (OSError, TimeoutError):
            en_menu = False
        time.sleep(1)
    else:
        # Sin reintentos: la ESP32 queda en el menú y el modo 8 no se activa más tarde.
        print(f"Sin batería de control en {VENTANA_S:.0f} s desde el arranque. El robot queda en el menú.")
        return

    # El sello precede al envío de '8'. Si el servicio falla, no se rearma solo.
    try:
        SELLO.touch(exist_ok=False)
    except FileExistsError:
        return
    try:
        indicacion = intercambiar(b"8", CONFIRMACION, 2)
        if b"8. Integracion" not in indicacion:
            raise RuntimeError("La ESP32 no confirmó la entrada a la prueba 8.")
        if PARO.exists():
            raise RuntimeError("Se solicitó el paro manual.")
        respuesta = intercambiar(b"SI\n", b"| BT1 ", 3)
        if b"-> Confirmado." not in respuesta:
            raise RuntimeError("La ESP32 no confirmó el modo de control.")
    except (RuntimeError, OSError, TimeoutError) as error:
        try:
            intercambiar(b"x", MENU, 1.5)
        except (OSError, TimeoutError):
            pass
        sys.exit(f"Modo 8 pendiente: {error}")
    print("Modo 8 seleccionado. El firmware comprueba el armado del control.")


if __name__ == "__main__":
    main()
