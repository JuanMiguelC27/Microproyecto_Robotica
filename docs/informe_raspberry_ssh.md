# Informe: Raspberry Pi 5 del proyecto Gecko

Fecha: 2026-11-19

---

## Parte A: información recopilada

### A1. Sistema

| Dato | Valor |
|---|---|
| Usuario | `jarvis` (uid 1000; grupos: jarvis, adm, cdrom, sudo, dip, video, plugdev, users, lpadmin, sambashare) |
| Hostname | `jarvis-desktop` |
| Sistema operativo | Ubuntu 24.04.4 LTS (Noble Numbat), kernel `6.8.0-1065-raspi`, aarch64 |
| Python | 3.12.3 |

Direcciones IP en el momento de la revisión:

```
lo     UNKNOWN  127.0.0.1/8 ::1/128
eth0   DOWN     (sin conexión)
wlan0  UP       11.11.7.52/24 fe80::8da:bd27:d301:4cb8/64   (conectada a WiFi-UAO)
```

### A2. Resolución por nombre

```
avahi-daemon 0.8-13ubuntu6.2   instalado
systemctl is-enabled avahi-daemon  -> enabled
systemctl is-active  avahi-daemon  -> active
```

El PC puede encontrar la Raspberry como **`jarvis-desktop.local`**.

### A3. SSH

```
openssh-server 1:9.6p1-3ubuntu13.20   instalado
ssh.service    enabled / active
ssh.socket     enabled / active
Escucha en     0.0.0.0:22 y [::]:22
```

- **Puerto:** 22.
- **`PasswordAuthentication`:** está comentada en `/etc/ssh/sshd_config` (línea 66), así que vale **`yes`** por defecto.
- **`PubkeyAuthentication`:** está comentada en `/etc/ssh/sshd_config` (línea 47), así que vale **`yes`** por defecto.
- `KbdInteractiveAuthentication no`.
- `/etc/ssh/sshd_config.d/` está vacío.
- No se pudo confirmar con `sshd -T` porque requiere sudo con contraseña.
- **`~/.ssh/authorized_keys`:** no existía (0 claves). `~/.ssh` existía con permisos `700`.

### A4. Wi-Fi

- **Gestor de red:** NetworkManager (renderer de netplan; `wpa_supplicant` actúa como backend de NM).
- **Interfaz inalámbrica:** `wlan0`.
- **Redes guardadas** (antes de la Parte B):

| Red | Autoconexión | Prioridad |
|---|---|---|
| WiFi-UAO (activa) | yes | 0 |
| David_ch_1195 | yes | 0 |
| David_collazos | yes | 0 |
| FLIA CASTANO DIAZ | yes | 0 |
| FLIA-RODRIGUEZ | yes | 0 |
| FLIA_DIAZ_ANGEL_Ext | yes | 0 |
| LABORATORIOS | yes | 0 |
| **MiniSumo-IMU** | yes | 0 |
| Robotica | yes | 0 |
| SEMILLERO_ROBOTICA | yes | 0 |
| Xiaomi 14T Pro | yes | 0 |

- `MiniSumo-IMU`: **guardada**.
- `Gecko-IMU`: **no estaba guardada** (se agregó en la Parte B).

### A5. Programa de la IMU

#### Carpeta y archivos

Carpeta: `/home/jarvis/mini_sumo_imu/` (**no es un repositorio git**).

```
.venv/                                      entorno virtual de Python
start_mini_sumo_pi.sh                       script de arranque (3558 B)
prompt_raspberry_ssh.md                     instrucciones de esta tarea
scripts/bno085_sender.py                    programa principal (158 líneas)
scripts/bno085_sender_antes_vibracion.py    versión anterior (116 líneas)
scripts/bno085_sender_backup.py             versión anterior (54 líneas)
scripts/bno085_i2c_test.py                  prueba de I2C (96 líneas)
scripts/__pycache__/                        caché de Python
```

#### Archivo principal y comando exacto

Archivo principal: `scripts/bno085_sender.py`. Se ejecuta **con el entorno virtual y sin `sudo`**. El script `start_mini_sumo_pi.sh` lo lanza con `exec`:

```bash
/home/jarvis/mini_sumo_imu/.venv/bin/python /home/jarvis/mini_sumo_imu/scripts/bno085_sender.py \
    --host 10.42.0.1 --port 5005 --rate 30
```

#### Cómo se inicia hoy

Solo **a mano**, con `./start_mini_sumo_pi.sh`. No hay ningún servicio de systemd, cron ni `rc.local` para la IMU. El script:

1. Conecta `wlan0` al hotspot `MiniSumo-IMU` con `nmcli`.
2. Hace ping a `10.42.0.1` hasta 30 veces y sale con error si no responde.
3. Inicia los servicios de usuario `vigilar-red-robotica.service` y `detector-grietas.service`. Se puede omitir con `INICIAR_DETECTOR=0`.
4. Ejecuta el emisor en primer plano.

#### Dónde se configuran la IP, el puerto y la frecuencia

- Valores por defecto en `scripts/bno085_sender.py`:

  ```python
  50  parser.add_argument("--host", default="10.42.0.1", help="PC hotspot IP")
  51  parser.add_argument("--port", type=int, default=5005)
  52  parser.add_argument("--rate", type=float, default=30.0, help="UDP send rate in Hz")
  ```

- El script de arranque los sobrescribe con variables de entorno en `start_mini_sumo_pi.sh`:

  ```bash
  11  PC_IP="${PC_IP:-10.42.0.1}"
  12  UDP_PORT="${UDP_PORT:-5005}"
  13  SEND_RATE="${SEND_RATE:-30}"
  ```

- Además, el programa envía el giroscopio (200 Hz, agrupado cada 50 ms) a `127.0.0.1:5006` (`--local-port`). Ese puerto lo lee el detector de grietas en `~/Detector_Grietas/detectar_remoto.py:33` (`PUERTO_IMU = 5006`).

#### Qué pasa si el PC no está disponible al arrancar

- **Script `start_mini_sumo_pi.sh`:** espera unos 30 s con ping y luego **se cierra con error** sin iniciar el emisor.
- **Programa Python:** **no se cierra**. Si `sendto` falla, imprime `UDP send error` como máximo cada 5 s y sigue funcionando (líneas 118-122). Como usa UDP, no detecta si el PC está escuchando.
- Si el BNO085 no responde al iniciar, la excepción no se captura y el programa termina con un traceback.

#### Cómo se detiene

- **`Ctrl+C`:** se captura `KeyboardInterrupt` (línea 145), imprime `BNO085 sender stopped.`, cierra el socket y libera el I2C en el bloque `finally`.
- **`SIGTERM` / `SIGHUP`:** **no se manejan**. El proceso termina de inmediato sin ejecutar `finally`. En la práctica es inofensivo, pero para una parada limpia conviene enviar `SIGINT`:

  ```bash
  pkill -INT -f bno085_sender.py
  ```

#### Registros

Solo escribe en la salida estándar (`print(..., flush=True)`). **No escribe archivos de registro.**

#### Menciones a "sumo" / "MiniSumo"

En los archivos `.py` no hay ninguna. Todas están en `start_mini_sumo_pi.sh`:

```
start_mini_sumo_pi.sh:3    # Uso: ./start_mini_sumo_pi.sh
start_mini_sumo_pi.sh:9    HOTSPOT_SSID="${HOTSPOT_SSID:-MiniSumo-IMU}"
start_mini_sumo_pi.sh:10   HOTSPOT_PASSWORD="${HOTSPOT_PASSWORD:-MiniSumo2026!}"
start_mini_sumo_pi.sh:15   VENV="${VENV:-$HOME/mini_sumo_imu/.venv}"
start_mini_sumo_pi.sh:16   SENDER="${SENDER:-$HOME/mini_sumo_imu/scripts/bno085_sender.py}"
start_mini_sumo_pi.sh:59   #    Para no iniciarlo: INICIAR_DETECTOR=0 ./start_mini_sumo_pi.sh
```

También el nombre de la carpeta del proyecto (`mini_sumo_imu`) y el del propio script (`start_mini_sumo_pi.sh`).

#### Contenido completo del archivo principal (`scripts/bno085_sender.py`, 158 líneas)

```python
#!/usr/bin/env python3
"""Lee el BNO085 y envía los datos por UDP a dos destinos.

1. PC (por el hotspot): cuaternión + aceleración lineal, el mismo paquete de siempre.
2. Página web del detector en esta Raspberry (127.0.0.1): cuaternión + TODAS las
   muestras del giroscopio (200 Hz) para calcular inclinación y frecuencia de vibración.
"""

import argparse
import json
import math
import socket
import time

import board
import busio
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C


class BNO085ConGiro(BNO08X_I2C):
    """La librería solo guarda el último dato de cada tipo y descarta el resto.
    Para medir vibraciones hace falta cada muestra del giroscopio, así que se guardan aquí."""

    def __init__(self, *args, **kwargs):
        self.muestras_giro = []
        super().__init__(*args, **kwargs)

    def _process_report(self, report_id, report_bytes):
        super()._process_report(report_id, report_bytes)
        if report_id == adafruit_bno08x.BNO_REPORT_GYROSCOPE:
            g = self._readings.get(report_id)
            if g is not None and all(math.isfinite(v) for v in g):
                self.muestras_giro.append([round(v, 5) for v in g])


def valid_vector(values, length):
    if values is None or len(values) != length:
        return None
    result = [float(v) for v in values]
    if not all(math.isfinite(v) for v in result):
        return None
    return result


def main():
    parser = argparse.ArgumentParser(
        description="Send BNO085 quaternion and linear acceleration to the PC over UDP."
    )
    parser.add_argument("--host", default="10.42.0.1", help="PC hotspot IP")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--rate", type=float, default=30.0, help="UDP send rate in Hz")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x4A,
                        help="BNO085 I2C address (default: 0x4A)")
    parser.add_argument("--local-port", type=int, default=5006,
                        help="Puerto UDP local de la página web (0 = no enviar)")
    parser.add_argument("--gyro-rate", type=float, default=200.0,
                        help="Frecuencia del giroscopio en Hz (máx. medible = la mitad). "
                             "200 Hz es lo más alto estable por I2C en la Raspberry.")
    args = parser.parse_args()

    if args.rate <= 0 or not math.isfinite(args.rate):
        parser.error("--rate must be a positive finite number")
    if not (1 <= args.port <= 65535):
        parser.error("--port must be between 1 and 65535")
    if not (0 < args.gyro_rate <= 200):
        parser.error("--gyro-rate must be between 0 and 200")

    i2c = None
    sock = None
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        sensor = BNO085ConGiro(i2c, address=args.address)

        REPORT_INTERVAL_US = 33333  # Aproximadamente 30 Hz
        sensor.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR, REPORT_INTERVAL_US)
        sensor.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION, REPORT_INTERVAL_US)
        sensor.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE, int(1_000_000 / args.gyro_rate))

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        period = 1.0 / args.rate
        periodo_local = 0.05  # 20 paquetes/s a la página web, cada uno con ~10 muestras del giroscopio
        sequence = 0
        proximo_pc = proximo_local = time.monotonic()
        ultimo_error_red = 0.0

        print(f"BNO085 initialized at I2C 0x{args.address:02X}", flush=True)
        print(f"Sending q + linear acceleration to {args.host}:{args.port} at {args.rate:g} Hz", flush=True)
        if args.local_port:
            print(f"Giroscopio a {args.gyro_rate:g} Hz -> página web (127.0.0.1:{args.local_port})", flush=True)
        print("Keep the robot still for the first calibration test; press Ctrl+C to stop.", flush=True)

        while True:
            # Se lee el sensor sin parar: si se espera entre lecturas, el BNO085
            # acumula reportes y se pierden muestras del giroscopio.
            try:
                sensor._process_available_packets()
            except (OSError, RuntimeError, ValueError, TypeError) as exc:
                print(f"BNO085 read error: {type(exc).__name__}: {exc}", flush=True)
                time.sleep(0.01)
                continue

            ahora = time.monotonic()
            q = valid_vector(sensor._readings.get(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR), 4)
            if q is not None:
                norm = math.sqrt(sum(v * v for v in q))
                q = [v / norm for v in q] if norm > 1e-9 and math.isfinite(norm) else None

            # --- Paquete al PC (sin cambios respecto a la versión anterior) ---
            if ahora >= proximo_pc:
                proximo_pc = max(proximo_pc + period, ahora - period)
                accel = valid_vector(sensor._readings.get(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION), 3)
                if q is not None and accel is not None:
                    packet = {"seq": sequence, "t_mono": ahora, "q": q, "a": accel}
                    payload = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
                    try:
                        sock.sendto(payload, (args.host, args.port))
                    except OSError as exc:
                        # Sin hotspot no se llega al PC, pero la página local sigue funcionando.
                        if ahora - ultimo_error_red > 5:
                            ultimo_error_red = ahora
                            print(f"UDP send error ({args.host}): {exc}", flush=True)
                    sequence += 1

                    if sequence % max(1, int(args.rate)) == 0:
                        print(
                            "seq={} q=({:+.3f},{:+.3f},{:+.3f},{:+.3f}) "
                            "a_lin=({:+.3f},{:+.3f},{:+.3f}) m/s^2".format(sequence, *q, *accel),
                            flush=True,
                        )

            # --- Paquete a la página web (inclinación + giroscopio completo) ---
            if ahora >= proximo_local:
                proximo_local = max(proximo_local + periodo_local, ahora - periodo_local)
                giro, sensor.muestras_giro = sensor.muestras_giro, []
                if args.local_port and (q is not None or giro):
                    payload = json.dumps({"t": ahora, "q": q, "g": giro}, separators=(",", ":")).encode()
                    try:
                        sock.sendto(payload, ("127.0.0.1", args.local_port))
                    except OSError:
                        pass

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nBNO085 sender stopped.", flush=True)
    finally:
        if sock is not None:
            sock.close()
        if i2c is not None:
            try:
                i2c.deinit()
            except Exception:
                pass


if __name__ == "__main__":
    main()
```

### A6. Hardware

- **I2C habilitado:** sí, `dtparam=i2c_arm=on` en `/boot/firmware/config.txt`.
- **Usuario en el grupo `i2c`:** **no** (`i2c:x:124:` sin miembros).
- El acceso actual a `/dev/i2c-1` se debe a una ACL `uaccess` (`user:jarvis:rw-`) que logind concede **solo mientras hay una sesión de escritorio activa** en `seat0` (tty2):

  ```
  crw-rw----+ 1 root i2c 89, 1 /dev/i2c-1
  user:jarvis:rw-
  ```

  > ⚠️ Si la Raspberry arranca sin pantalla y nadie inicia sesión, lo más probable es que el programa lanzado por SSH falle con `Permission denied` al abrir el I2C. Ver el paso B5.

- **`i2cdetect -y 1`:** el BNO085 responde en `0x4a`.

  ```
       0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
  00:                         -- -- -- -- -- -- -- --
  10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  40: -- -- -- -- -- -- -- -- -- -- 4a -- -- -- -- --
  50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
  70: -- -- -- -- -- -- -- --
  ```

### A7. Servicios relacionados

- **Servicios de usuario** en `~/.config/systemd/user/` (linger activado: `Linger=yes`):

  | Unidad | Estado | Función |
  |---|---|---|
  | `vigilar-red-robotica.service` | enabled, `Restart=always` | Ejecuta `~/Detector_Grietas/iniciar_remoto.sh`, que mantiene el detector encendido mientras haya red (ngrok solo en la red Robotica) |
  | `detector-grietas.service` | disabled (lo arranca el anterior), `Restart=always` | Servidor web en el puerto 5000; recibe la IMU por UDP 5006 |
  | `ngrok-detector.service` | — | Túnel ngrok para el detector |

- **Unidades de sistema** relacionadas con la IMU o el proyecto: ninguna.
- **cron:** `no crontab for jarvis`. En `/etc/cron.d` solo hay `anacron`, `e2scrub_all` y `sysstat`.
- **`/etc/rc.local`:** no existe.
- **Autostart / `.bashrc` / `.profile`:** sin referencias a la IMU.

---

## Parte B: cambios realizados

| Paso | Estado | Detalle |
|---|---|---|
| B1. SSH al arrancar | Sin cambios (ya estaba) | `ssh.service` y `ssh.socket` ya estaban habilitados y activos |
| B2. Red `Gecko-IMU` | ✅ Hecho | Ver abajo |
| B3. Clave pública del PC | ✅ Hecho | Ver abajo |
| B4. avahi-daemon | Sin cambios (ya estaba) | Ya estaba habilitado y activo |
| B5. Grupo `i2c` | ⏳ Pendiente | Necesita `sudo` con contraseña |
| Programa de la IMU | Sin cambios | No se tocó el programa ni la forma de iniciarlo |

### B2. Red `Gecko-IMU`

Comando ejecutado:

```bash
nmcli connection add type wifi ifname wlan0 con-name Gecko-IMU ssid Gecko-IMU \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk '********' \
    connection.autoconnect yes connection.autoconnect-priority 10
```

Resultado:

```
Gecko-IMU:    autoconnect=yes  prioridad=10  key-mgmt=wpa-psk  psk-flags=0  permissions=(todas)
MiniSumo-IMU: autoconnect=yes  prioridad=0   (sin cambios)
```

- La clave queda guardada a nivel del sistema (`psk-flags=0`) y la conexión no está limitada a un usuario. La Pi se conecta sola al arrancar, aunque nadie inicie sesión.
- No se borró ninguna red. Ahora hay 12 conexiones Wi-Fi guardadas.
- Con prioridad 10, si `Gecko-IMU` está al alcance, se prefiere sobre WiFi-UAO y las demás redes (prioridad 0).

### B3. Clave pública del PC

Comandos ejecutados:

```bash
KEY='ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIAyynafDK2ijZdlSB/O9Idwxy04HJqbrCwABVfJGdmvp juanmiguel.castanod@gmail.com'
mkdir -p ~/.ssh && chmod 700 ~/.ssh
touch ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys
grep -qxF "$KEY" ~/.ssh/authorized_keys || echo "$KEY" >> ~/.ssh/authorized_keys
```

Resultado:

```
drwx------  jarvis jarvis  /home/jarvis/.ssh
-rw-------  jarvis jarvis  /home/jarvis/.ssh/authorized_keys   (1 clave)
```

### B5. Pendiente: acceso permanente a I2C

Hay que ejecutarlo a mano porque `sudo` pide contraseña. Se aplica después de reiniciar:

```bash
sudo usermod -aG i2c jarvis
sudo reboot
```

Para comprobarlo después del reinicio:

```bash
id jarvis | grep i2c
```

---

## Verificación desde el PC

Con el hotspot `Gecko-IMU` activo, este comando debería responder `ok` sin pedir contraseña:

```bash
ssh jarvis@jarvis-desktop.local 'echo ok'
```

Detener el programa de forma limpia (SIGINT, equivale a Ctrl+C):

```bash
ssh jarvis@jarvis-desktop.local 'pkill -INT -f bno085_sender.py'
```

**Comando que el PC debe ejecutar por SSH para iniciar el programa:**

`ssh jarvis@jarvis-desktop.local '~/mini_sumo_imu/.venv/bin/python -u ~/mini_sumo_imu/scripts/bno085_sender.py --host 10.42.0.1 --port 5005 --rate 30'`
