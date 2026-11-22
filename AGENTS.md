# Gecko: contexto del proyecto para Codex

Responde siempre en español. Fecha de este documento: 2026-11-20.

Gecko es el vehículo de inspección del Microproyecto de Robótica. Este archivo da a Codex el mismo contexto y las mismas capacidades que tenía la sesión de Claude Code: levantar el hotspot, entrar a la Raspberry por SSH y manejar la ESP32 por el puerto serial.

---

## 1. Forma de trabajo con agentes

El agente principal es el **líder del proyecto**. Planifica, reparte el trabajo, revisa lo que entregan los subagentes y verifica los resultados antes de dar algo por terminado. Delega en dos subagentes:

| Rol | Modelo en Codex | Equivalente en Claude | Tarea |
|---|---|---|---|
| Líder (sesión principal) | `gpt-6.2-astra`, razonamiento alto | Opus 5.5 | Planificar, decidir, revisar, ejecutar comandos en el robot |
| **Reader** | `gpt-6-luna`, razonamiento bajo | Haiku 5.5 | Leer documentación, código y conversaciones anteriores, solo lectura |
| **Programmer** | `gpt-6.2-sol`, razonamiento medio | Sonnet 5.5 | Escribir y modificar **todo** el código |

Reglas:
- El líder **no escribe código**. Solo interviene si Programmer se queda dando vueltas, es decir, si repite intentos sin avanzar.
- Para un mismo tema, sigue con el mismo subagente en lugar de crear uno nuevo, para que no pierda el contexto.
- Reader nunca modifica archivos.
- En cada encargo a Programmer, incluye las reglas de seguridad de la sección 2.

### Configuración (una sola vez, en `~/.codex/config.toml`)

```toml
model = "gpt-6.2-astra"
model_reasoning_effort = "high"

[features]
multi_agent_v2 = true

[agents.reader]
description = "Reader: lee documentación, código y conversaciones previas. Solo lectura, nunca modifica archivos."
config_file = "agents/reader.toml"

[agents.programmer]
description = "Programmer: genera y modifica todo el código del proyecto Gecko."
config_file = "agents/programmer.toml"
```

`~/.codex/agents/reader.toml`:
```toml
model = "gpt-6-luna"
model_reasoning_effort = "low"
sandbox_mode = "read-only"
```

`~/.codex/agents/programmer.toml`:
```toml
model = "gpt-6.2-sol"
model_reasoning_effort = "medium"
```

Los nombres de modelo salen de `~/.codex/models_cache.json` en codex-cli 0.171.0. Si cambian, usa la misma idea: el modelo más capaz para el líder, el rápido y barato para Reader, y el de programación para Programmer. Comprueba que `multi_agent_v2` quedó activo con `codex features list`.

### Conversaciones anteriores (para Reader)

Las sesiones de Claude Code están en `~/.claude/projects/-home-juanm-mk/*.jsonl`, con una entrada JSON por línea. Hay que buscar en ellas con `grep`, no leerlas enteras. La memoria resumida de Claude está en `~/.claude/projects/-home-juanm-mk/memory/`, en los archivos `gecko-*.md`.

---

## 2. Reglas de seguridad (obligatorias)

1. **Firmware de la ESP32:** no modificar ni flashear `GeckoBuddy_PruebasPCB/`. Si un cambio parece necesario:
   - Primero explica al usuario qué quieres cambiar y por qué.
   - Espera su aprobación.
   - Haz una copia de respaldo antes de modificarlo, por ejemplo `cp -a GeckoBuddy_PruebasPCB GeckoBuddy_PruebasPCB_respaldo_AAAAMMDD`.
2. **Antes de cualquier comando que mueva actuadores** (`4`, `6`, `7` u `8`), confirma con el usuario que:
   - el robot está sujeto, con las ruedas en el aire;
   - las hélices de las EDF están libres;
   - el control está encendido, con VrA y VrB abajo y los sticks centrados.
3. **Sin baterías no se mueve nada.** Si `9` muestra las baterías como `NO CONECTADA`, la ESP32 se alimenta solo por USB: usa únicamente comandos de lectura (`1`, `2`, `5`, `9`).
4. **Paro de emergencia:** `esp32ctl.sh send x`, en cualquier momento.
5. **`sudo` en la Raspberry pide contraseña.** Codex no puede escribirla: pídele al usuario que ejecute en otra terminal `ssh -t jarvis@10.42.0.118 'sudo …'`.
6. **No hacer push, commits, borrados ni cambios de configuración del sistema** sin pedirlo al usuario.

---

## 3. Red: PC → hotspot → Raspberry → ESP32

```
PC (wlo1, hotspot Gecko-IMU 10.42.0.1)
   │ Wi-Fi, canal 11
Raspberry Pi 5  jarvis@10.42.0.118  (hostname jarvis-desktop, Ubuntu 24.04)
   │ cable USB
ESP32 DevKit V1  /dev/ttyUSB0  (CP2102, 115200 baud)
```

### Conexión a internet del PC
El hotspot ocupa `wlo1`, la única Wi-Fi del PC, y `eno1` no tiene cable. **Antes de levantar el hotspot**, el usuario conecta el celular por anclaje USB (interfaz `enx…`). Si no lo hace, el PC pierde internet y Codex también. Comprueba la ruta por defecto:
```bash
ip route | grep default     # debe existir una ruta "dev enx..." con métrica menor que wlo1
```

### Levantar el hotspot
La conexión `Gecko-IMU` ya existe en NetworkManager: modo AP, canal 11, sin ahorro de energía, IP 10.42.0.1/24.
```bash
nmcli connection up Gecko-IMU
ip -br addr show wlo1                      # debe mostrar 10.42.0.1/24
iw dev wlo1 station dump | grep Station    # dispositivos conectados al hotspot
nmcli connection down Gecko-IMU            # apagarlo (wlo1 vuelve a WiFi-UAO)
```
Si la conexión no existe, `IMU/gecko_imu/start_gecko_pc.sh` la crea, con el SSID y la contraseña definidos en el propio script. Ese script además arranca todo el gemelo digital. Para levantar solo el hotspot, usa `nmcli`.

**Problemas conocidos:**
- **La Raspberry no se cambia de red** si ya está conectada a otra (por ejemplo WiFi-UAO). Hay que desenergizarla y volver a energizarla **después** de levantar el hotspot.
- **Error "Driver does not support AP mode":** pasa si se recargó el driver rtw89 del PC. Se arregla con `sudo systemctl restart wpa_supplicant`, que debe ejecutar el usuario.
- **Latencias de unos 800 ms:** las causaba el ahorro de energía de rtw89. Ya está resuelto de forma permanente en `/etc/modprobe.d/rtw89-gecko.conf`; no hace falta repetirlo.

### Esperar a la Raspberry y entrar por SSH
La clave SSH del PC ya está autorizada, así que no pide contraseña.
```bash
for i in $(seq 1 60); do timeout 6 ssh -o ConnectTimeout=4 -o BatchMode=yes jarvis@10.42.0.118 true && { echo OK; break; }; sleep 5; done
ssh jarvis@10.42.0.118 'hostname; ls -l /dev/ttyUSB0'
```
Desde la red de la universidad no se alcanza la Raspberry: solo funciona por el hotspot. `jarvis` ya está en los grupos `dialout` (puerto serial) e `i2c` (IMU).

---

## 4. Manejar la ESP32: puente serial

Si se cierra y se vuelve a abrir el puerto, la ESP32 se reinicia. Por eso hay un puente que deja el puerto abierto y recibe los comandos por un FIFO.
- **En la Raspberry:** `~/gecko_tools/esp32_bridge.py` y `~/gecko_tools/esp32ctl.sh`.
- **Copia en el PC:** `IMU/gecko_imu/scripts/pi_remote/`. Para actualizarla en la Raspberry: `scp esp32_bridge.py esp32ctl.sh jarvis@10.42.0.118:gecko_tools/`.
- **Archivos de trabajo, en la Raspberry:** `~/gecko_serial/esp32.log`, `cmd.fifo` y `bridge.pid`.

```bash
E='ssh jarvis@10.42.0.118 ~/gecko_tools/esp32ctl.sh'
$E start           # abre el puerto (la ESP32 puede reiniciarse al abrir; arranca en estado seguro)
$E status
$E run 9 2         # envía "9" sin Enter, espera 2 s e imprime solo lo nuevo
$E runl SI 4       # envía "SI\n", para las confirmaciones
$E send x          # envía una tecla sin esperar respuesta (paro)
$E tail 60         # últimas líneas del log
$E stop            # envía 'x' (paro) y cierra el puerto
```
### Arranque automático (instalado 2026-11-20, servicios de usuario con linger)
- `gecko-serial.service`: el puente arranca solo al encender la Raspberry. `esp32ctl.sh start/stop` lo manejan con systemd.
- `gecko-control.service` (`~/gecko_tools/gecko_mode8_boot.py`): una vez por arranque, si en los primeros 180 s la prueba `9` muestra la **2S de control conectada**, envía `8` + `SI`. El armado por sticks y el failsafe del firmware siguen vigentes. No reintenta después de 180 s. `esp32ctl.sh send x` o `stop` crean `$XDG_RUNTIME_DIR/gecko-paro-manual` y bloquean el modo 8 automático hasta el siguiente reinicio.
- `gecko-imu.service`: emisor BNO085 (UDP a 10.42.0.1:5005 y a la página por 127.0.0.1:5006). `start_sender.sh`/`stop_sender.sh` paran este servicio antes de lanzar o detener el emisor manual.
- Copias locales de todo en `IMU/gecko_imu/scripts/pi_remote/`.

---

## 5. Comandos seriales del firmware

El menú usa teclas de un solo carácter, sin Enter. Las confirmaciones (`SI`) y los números llevan Enter. Dentro de cada prueba, `x` o `q` salen.

| Tecla | Prueba | Detalle |
|---|---|---|
| `1` | ESP32 y D12 | Chip, memoria, MAC, D12 a GND y mapa de pines. Solo lectura. |
| `2` | Baterías | Lecturas cada segundo. `c` calibra (pide el número de batería y la tensión del multímetro). |
| `3` | STBY y LED | `s` alterna STBY, `p` parpadeo de 10 s. |
| `4` | Motores TB6612 | Pide `SI`. `1`–`4` o `t` lanzan secuencias. `m` es modo manual: `1`–`4` elige motor, `w`/`s` ±10 %, `b` freno, `c` libre. |
| `5` | Receptor i-BUS | CH1–CH6 cada 250 ms, más tramas buenas y malas. Solo lectura. |
| `6` | Servos | Pide `SI`. `c` centro, `b` barrido, `l`/`r`/`a` elige servo, `+`/`-` ±50 µs. |
| `7` | ESC/EDF | Pide `SI` y luego el número de ESC con Enter. `+`/`-` ±25 µs hasta 1300 µs, `0` apaga. Si pasan 10 s sin teclas, vuelve a 1000 µs. |
| **`8`** | **Todo con el control** | Pide `SI`. Ver abajo. |
| `9` | Estado de todo | D12, STBY, 3 baterías, tope de PWM, motores, ESC, servos e i-BUS. Solo lectura. |
| `x` / espacio | Paro global | STBY bajo, motores frenados, ESC a 1000 µs. |

### Comando 8 en detalle
- **Control:** FlySky FS-i6X con receptor FS-iA6B, conectado por **i-BUS** (puerto *i-BUS SERVO* a J1-3, GPIO34, 115200 baudios). No hay Bluetooth. El control debe estar en *RX Setup → Output mode → Serial i-BUS*.
- **Armado:** CH3 (avance, stick izquierdo **sin resorte**) y CH4 (giro) en 1500 ± 60 µs durante 1 s. Aparece `>> ARMADO: STBY alto`.
- **Ruedas en modo tanque:**
  - izquierda = CH3 + CH4 (M1 y M3);
  - derecha = CH3 − CH4 (M2 y M4, invertidos);
  - zona muerta de 80 µs y rampa de 2,5 por segundo.
- **Servos:** CH2 y CH1.
- **EDF:** CH5 mueve ESC1 (izquierda) y CH6 mueve ESC2 (derecha). Se habilitan cuando VrA y VrB están a 1050 µs o menos a la vez.
- **Failsafe:** si pasan más de 100 ms sin trama, todo frena, STBY baja y los ESC van a 1000 µs. La línea de telemetría empieza con `SIN`.
- **Telemetría cada 500 ms:**
  `ARM OK | CH c1 c2 c3 c4 c5 c6 | rueda izq % der % (tope %) | ESC a/b | servos a/b | BT1 V 2S V 3S V`
  - `BT1` es la batería 2S de **control** (no Bluetooth).
  - `2S` es la de tracción y `3S` la de las EDF.
- **Salir:** `x`, espacio o `q`.

### Observaciones (sin corregir, porque implican tocar el firmware)
1. **El firmware cargado no coincide exactamente con el código fuente.** La placa muestra un tope de 97 % con la batería de tracción a 8,23 V. El código daría 73 % (`V_MOTOR = 6.0`), así que en la placa el límite parece estar en unos 8 V. Si los motores son de 6 V, pueden recibir hasta unos 8 V.
2. **Las EDF no se vuelven a bloquear tras un failsafe** (`tests.cpp`, la variable `edf` nunca vuelve a `false`). Si el control se reconecta con VrA o VrB arriba, las EDF aceleran solas.
3. **Las EDF reciben 900 µs** (CH5 y CH6 al mínimo), que el firmware no recorta a 1000. Se puede ajustar en los extremos de canal del control, sin tocar el firmware.
4. **CH3 en reposo marca unos 1433 µs.** Queda dentro de la zona muerta, pero conviene centrar el trim.
5. **No hay corte automático por batería baja**, solo avisos en pantalla.

### Último estado conocido (2026-11-20)
- **Baterías:** 2S de control a 8,23 V y 2S de tracción a 8,23 V; 3S desconectada.
- **Receptor:** i-BUS funcionando, sin tramas malas.
- **Control:** el modo 8 movió las ruedas correctamente.

---

## 6. Resto del proyecto

| Qué | Dónde |
|---|---|
| Firmware ESP32 (PlatformIO, **no tocar**) | `GeckoBuddy_PruebasPCB/` (`include/config.h` tiene los pines; `src/tests.cpp` tiene las pruebas; `LEEME.md`) |
| Gemelo digital (ROS 2, Gazebo Fortress y RViz) | `IMU/gecko_imu/`; se arranca con `./start_gecko_pc.sh`, `stop` para cerrar y `rviz` para reabrir RViz |
| URDF fuente | `Micro2_URDFV2/` |
| Documentación e informes | `docs/` (por ejemplo `docs/informe_raspberry_ssh.md`) |
| Emisor de la IMU en la Raspberry (BNO085, UDP hacia 10.42.0.1:5005, unos 29 Hz) | `~/mini_sumo_imu/scripts/bno085_sender.py` con su `.venv` |
| Cámara y detector de grietas | Página `http://10.42.0.118:5000` (vídeo, IMU, baterías y telemetría del modo 8 leídas de `esp32.log`), servicio de usuario `detector-grietas`, archivo `~/Detector_Grietas/detectar_remoto.py`. Copias en `docs/respaldo_camara_20261120/`. Para subirlo: `scp` a un temporal, comparar `sha256sum` y luego `mv` (una copia cortada dejó la página caída el 2026-11-20). |

Notas del entorno del PC:
- Para ROS usa `/usr/bin/python3`, porque el `python3` del PATH es Miniconda.
- PlatformIO y Pinocchio están en otros entornos. Consulta la memoria de Claude antes de usarlos.

Notas de la cámara:
- Es una Arducam IMX519 en el puerto CAM/DISP 0.
- Si un kernel nuevo da `No cameras available!`, hay que recompilar el módulo de enfoque: `sudo bash ~/src/ak7375/reinstalar_ak7375.sh`, que debe ejecutar el usuario.
- El kernel está fijado con `apt-mark hold linux-image-raspi`.
