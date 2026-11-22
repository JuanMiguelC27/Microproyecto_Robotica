# Gecko Buddy — pruebas modulares de la PCB

Programa para la **ESP32 DevKit V1 de 30 pines** de la placa principal. Prueba por etapas cada parte de la placa principal y de la PCB-C con un menú en el monitor serie. La lista completa de conexiones está en la pestaña "Conexiones y pruebas de la PCB" del documento de conexiones.

**D12 (GPIO12) está unido a GND en la PCB.** El programa lo deja siempre como entrada. Solo lo lee para comprobar el puente, con el pull-up interno, lo cual es seguro. No lo cambies a salida.

## 1. Preparar Ubuntu 22.04 (una sola vez)

1. Instala lo necesario y VS Code:
   ```bash
   sudo apt update
   sudo apt install -y python3-venv curl
   sudo snap install code --classic
   ```
2. En VS Code abre **Extensiones** (Ctrl+Shift+X), busca **PlatformIO IDE** e instálala. Espera a que termine; abajo a la izquierda aparece el ícono de la hormiga.
3. Permiso para usar el puerto USB:
   ```bash
   sudo usermod -aG dialout $USER
   curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
   Luego **cierra sesión y vuelve a entrar** (o reinicia) para que el grupo `dialout` tenga efecto.
4. Si tu ESP32 usa el chip **CH340** y el puerto `/dev/ttyUSB0` aparece y desaparece, en Ubuntu 22.04 lo está tomando el programa `brltty`. Quítalo:
   ```bash
   sudo apt remove -y brltty
   ```
5. Conecta la ESP32 y comprueba que aparece el puerto:
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```

## 2. Abrir, cargar y usar

1. En VS Code: **File → Open Folder** → elige la carpeta `GeckoBuddy_PruebasPCB`.
   - La primera vez PlatformIO descarga la plataforma ESP32 (unos minutos).
2. Barra azul de abajo:
   - **✓ (Build):** compila.
   - **→ (Upload):** carga el programa en la ESP32. Si se queda en `Connecting....`, mantén presionado el botón **BOOT** de la tarjeta hasta que empiece a escribir.
   - **Enchufe (Monitor):** abre el monitor serie a 115200.
3. En el monitor aparece el menú. Escribe el número de la prueba. Cuando una prueba pide confirmación, escribe `SI` y presiona Enter.

### Un programa por sección

El proyecto tiene 9 programas (entornos de PlatformIO). Se elige cuál cargar en la barra azul de abajo, en el botón `env:...`, o en la barra lateral de PlatformIO → **Project Tasks** → nombre del entorno → **Upload and Monitor**.

| Entorno | Qué hace |
| --- | --- |
| `menu_completo` | menú con las 9 pruebas (el de siempre) |
| `p1_esp32` | prueba 1: ESP32 y D12 |
| `p2_baterias` | prueba 2: baterías y calibración |
| `p3_stby_led` | prueba 3: STBY y LED |
| `p4_motores` | prueba 4: motores |
| `p5_ibus` | prueba 5: receptor |
| `p6_servos` | prueba 6: servos |
| `p7_esc_edf` | prueba 7: ESC y EDF |
| `p8_integracion` | prueba 8: todo junto |

Los programas `p1`–`p8` arrancan directo en su prueba. Al salir con `q`: `r` la repite, `9` muestra el estado y `x` es el paro.

Desde la terminal también sirve: `~/.platformio/penv/bin/pio run -e p4_motores -t upload -t monitor`.

**Paro inmediato en cualquier prueba:** tecla `x` o barra espaciadora. Pone STBY en bajo, frena los motores y manda 1000 µs a los ESC. Con `q` se vuelve al menú.

## 3. Pruebas y etapas de montaje

| # | Prueba | Qué se conecta antes | Qué debe pasar |
| --- | --- | --- | --- |
| 1 | ESP32 y D12 | Solo la ESP32 por USB | `D12 = GND OK`, datos del chip y mapa de pines |
| 2 | Baterías y divisores | 2S de control en J2, 2S de tracción en J6; luego la 3S | Cada valor dentro de ±0,1 V del multímetro; `c` calcula la calibración |
| 3 | STBY y LED | Igual que 1 o 2 | El LED prende y apaga con `s`; 3,3 V / 0 V en el pin STBY de cada TB6612 |
| 4 | Drivers y motores | Módulos TB6612, motores en J8–J11, **ruedas en el aire** | Cada rueda: frena, adelante 30/60 % y tope, freno corto, atrás, rueda libre |
| 5 | Receptor i-BUS | Receptor en J1, control en Serial i-BUS | ~143 tramas/s, 0 malas, canales de 1000 a 2000 |
| 6 | Servos | PCB-C por W1, servos en J5/J6 de la PCB-C, 5 V en +5V_SERVO | Centro, barrido y ajuste de a 50 µs sin forzar |
| 7 | ESC y EDF | 3S en J7 de la PCB-C, ESC en J3/J4 de la PCB-C, **robot y EDF sujetos** | Arman con pitidos; `+`/`-` suben y bajan hasta 1300 µs; a los 10 s sin teclas vuelve a 1000 |
| 8 | Integración | Todo | Arma con CH3/CH4 al centro, ruedas y servos siguen las palancas, EDF bloqueadas hasta bajar VrA y VrB, freno al apagar el control |
| 9 | Estado | — | Resumen de D12, STBY, baterías, salidas e iBUS |

## 4. Ajustes en `include/config.h`

- **Divisores:** `DIV_CTRL`, `DIV_TRAC`, `DIV_PROP`. Están en 100k/39k, 100k/39k y 100k/22k. Tu KiCad dice R2 = 33k: si esa es la que sueldas, cambia `DIV_CTRL` a `(100.0f + 33.0f) / 33.0f`.
- **Calibración:** `CAL_CTRL`, `CAL_TRAC`, `CAL_PROP`. La prueba 2 (tecla `c`) te da la línea exacta para pegar.
- **Motores:** pines, nombre e `invert` de cada uno. Si una rueda gira al revés, cambia su `invert` o intercambia los dos cables del motor.
- **Tope de la prueba de ESC:** `ESC_TEST_LIMIT_US` (1300 µs por defecto).

## 5. Si algo falla

| Síntoma | Revisa |
| --- | --- |
| No aparece `/dev/ttyUSB0` | Cable USB de datos; `brltty` (paso 1.4); otro puerto USB |
| `Permission denied` al cargar | Grupo `dialout` y volver a iniciar sesión |
| Prueba 1: `FALLA` en D12 | Soldadura del puente D12–GND |
| Prueba 2: batería en 0 V o `NO CONECTADA` | Fusible e interruptor de esa batería, conector J2/J6, R1–R4 |
| Prueba 4: un motor no gira | VM (J7-1 debe dar el voltaje de la 2S), STBY en 3,3 V, GND del módulo, conector del motor |
| Prueba 4: un motor solo gira hacia un lado | IN1 o IN2 de ese módulo sin soldar |
| Prueba 5: "No llega NADA" | GND común, J1-3 a S del puerto i-BUS SERVO, receptor enlazado |
| Prueba 5: "Llegan bytes pero ninguna trama válida" | Control en Serial i-BUS; puerto SERVO, no SENS |
| Prueba 6: servos quietos | 5 V en J3-2 de la PCB-C (BEC del ESC1 o fuente provisional) |
| Prueba 7: el ESC no arma | 3S y S3; que el ESC reciba 1000 µs al encender (la prueba los manda) |

El LED azul de la tarjeta está en D2 (M4_IN2): se prende cuando trabaja el motor 4.

## 6. Archivos

- `include/config.h`: pines y valores.
- `src/actuators.*`: motores, STBY, servos y ESC (y paro total).
- `src/battery.*`: baterías.
- `src/ibus.*`: lector i-BUS.
- `src/console.*`: monitor serie.
- `src/tests.*`: las 9 pruebas.
- `src/main.cpp`: menú.
- `src/pwm_out.h`: PWM que compila con Arduino-ESP32 2.x y 3.x.
- `platformio.ini`: fija `espressif32@6.9.0` (Arduino-ESP32 2.0.17).
