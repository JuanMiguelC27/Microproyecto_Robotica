"""Detector de grietas con vista remota.

Corre en la Raspberry y publica una página web con el vídeo en vivo y las alertas.
Desde el computador abre en el navegador:  http://<IP-de-la-Raspberry>:5000
"""
import json
import math
import multiprocessing
import os
import socket
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, abort, jsonify, send_from_directory

# --- Ajustes ---
CARPETA = Path(__file__).resolve().parent
TAMANO = 640              # 640 = más preciso | 480 = más rápido
CONFIANZA = 0.15          # confianza mínima para marcar una grieta
DETECCIONES_SEGUIDAS = 2  # detecciones seguidas necesarias para avisar (evita falsas alarmas)
ESPERA_CAPTURAS = 5       # segundos mínimos entre fotos guardadas
PUERTO = 5000
CALIDAD_JPEG = 55         # calidad de la imagen con grietas dibujadas (más bajo = menos datos)
ANCHO_WEB = 640           # ancho de la imagen enviada (la detección usa siempre la imagen completa)
FPS_CAMARA = 30           # la imagen se envía apenas llega: a más FPS, menos latencia
# Modo del sensor IMX519 que usa el sensor completo (binning 2x2). Sin él, rpicam-vid
# elige 1280x720, que recorta el centro y reduce mucho el campo de visión.
MODO_SENSOR = "2328:1748:10:P"

# --- IMU (BNO085): llega desde bno085_sender.py por UDP local ---
PUERTO_IMU = 5006         # debe coincidir con --local-port del emisor
VENTANA_VIBRACION = 4.0   # segundos de giroscopio usados en cada cálculo (resolución ≈ 1/4 = 0.25 Hz)
FREC_MIN_VIBRACION = 1.0  # Hz; por debajo son giros/maniobras del vehículo, no vibración
UMBRAL_VIBRACION = 0.5    # °/s RMS; por debajo se considera que no vibra (solo ruido del sensor)

CAPTURAS = CARPETA / "capturas"
CAPTURAS.mkdir(exist_ok=True)

MODELOS_NCNN = {
    640: CARPETA / "best_ncnn_model",
    480: CARPETA / "best480_ncnn_model",
    416: CARPETA / "best416_ncnn_model",
}

# La detección corre en otro proceso, en estos núcleos y con menos prioridad, para que
# el vídeo (este proceso) no espere al modelo: en un mismo proceso ambos compiten por
# el GIL de Python y el vídeo caía a ~8 FPS.
NUCLEOS_DETECTOR = {1, 2, 3}


def cargar_modelo():
    from ultralytics import YOLO
    ruta_ncnn = MODELOS_NCNN.get(TAMANO)
    if ruta_ncnn and ruta_ncnn.exists():
        return YOLO(str(ruta_ncnn), task="segment")
    import torch
    torch.set_num_threads(len(NUCLEOS_DETECTOR))
    return YOLO(str(CARPETA / "best.pt"))


def iniciar_camara():
    return subprocess.Popen(
        [
            "rpicam-vid",
            "-n", "-t", "0",
            "--codec", "mjpeg",
            "--width", "640",
            "--height", "480",
            "--mode", MODO_SENSOR,
            "--framerate", str(FPS_CAMARA),
            "--flush",
            "--tuning-file", str(CARPETA / "imx519_af.json"),
            "--autofocus-mode", "continuous",
            "--autofocus-range", "normal",
            "-o", "-",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    )


camara = None

# --- Estado compartido entre hilos ---
candado = threading.Lock()
nuevo_frame = threading.Condition(candado)  # avisa al vídeo apenas llega una imagen
activo = True
ultimo_jpg = None          # JPEG tal como sale de la cámara
hora_captura = 0.0         # time.time() cuando la imagen terminó de llegar de la cámara
id_frame = 0
ultimo_resultado = None
id_resultado = 0
cache_web = (None, None)   # ((id_frame, id_resultado), JPEG) para no recodificar la misma imagen
estado = {
    "grietas": 0,
    "confianza_max": 0.0,
    "alerta": False,
    "detecciones_por_seg": 0.0,
    "total_alertas": 0,
}
historial = deque(maxlen=20)
estado["imu"] = None  # se llena con los datos del BNO085 cuando lleguen


# --- IMU: inclinación y frecuencia de vibración ---
def angulos_euler(q):
    """Cuaternión (x, y, z, w) del BNO085 -> roll, pitch, yaw en grados.
    Roll = giro sobre X, pitch = giro sobre Y, yaw = giro sobre Z (rumbo)."""
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [round(math.degrees(a), 1) for a in (roll, pitch, yaw)]


def analizar_vibracion(muestras, fs):
    """Frecuencia dominante de vibración a partir de la velocidad angular (giroscopio).

    La velocidad angular es la derivada de la inclinación: si el vehículo vibra, su
    inclinación oscila y el giroscopio ve esa oscilación con mucha más resolución.
    Se busca el pico del espectro (FFT) sumando la energía de los 3 ejes."""
    g = np.degrees(np.asarray(muestras, dtype=float))          # N x 3, en °/s
    n = len(g)
    t = np.arange(n)
    # Quitar la parte lenta (giros, inclinación sostenida) con una recta por eje
    for eje in range(3):
        g[:, eje] -= np.polyval(np.polyfit(t, g[:, eje], 1), t)
    rms_ejes = np.sqrt(np.mean(g ** 2, axis=0))
    rms = float(np.sqrt(np.sum(rms_ejes ** 2)))
    resultado = {
        "intensidad": round(rms, 2),
        "eje": "XYZ"[int(np.argmax(rms_ejes))],
        "frecuencia": None,
        "amplitud": None,
    }
    if rms < UMBRAL_VIBRACION:
        return resultado

    ventana = np.hanning(n)
    espectro = np.sum(np.abs(np.fft.rfft(g * ventana[:, None], axis=0)) ** 2, axis=1)
    frecs = np.fft.rfftfreq(n, 1.0 / fs)
    validos = np.where(frecs >= FREC_MIN_VIBRACION)[0]
    if len(validos) < 3:
        return resultado
    k = validos[np.argmax(espectro[validos])]
    # Interpolación parabólica del pico para afinar más allá de la resolución de la FFT
    if 0 < k < len(espectro) - 1:
        a, b, c = np.log(espectro[k - 1:k + 2] + 1e-12)
        desplazamiento = 0.5 * (a - c) / (a - 2 * b + c) if (a - 2 * b + c) != 0 else 0.0
    else:
        desplazamiento = 0.0
    frecuencia = float((k + desplazamiento) * fs / n)
    resultado["frecuencia"] = round(frecuencia, 2)
    # Amplitud de la oscilación de la inclinación: ángulo = ω / (2πf); pico = RMS·√2
    resultado["amplitud"] = round(rms * math.sqrt(2) / (2 * math.pi * frecuencia), 3)
    return resultado


def leer_imu():
    """Recibe los paquetes del emisor del BNO085 y actualiza estado["imu"]."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", PUERTO_IMU))
    sock.settimeout(1.0)
    giro = deque()          # paquetes (tiempo_llegada, [[gx, gy, gz], ...]) en rad/s
    angulos = None
    ultimo_dato = 0.0
    ultimo_calculo = 0.0
    vibracion = {}
    while activo:
        ahora = time.monotonic()
        try:
            datos, _ = sock.recvfrom(65535)
            paquete = json.loads(datos)
            ahora = time.monotonic()
            ultimo_dato = ahora
            if paquete.get("q"):
                angulos = angulos_euler(paquete["q"])
            if paquete.get("g"):
                giro.append((ahora, paquete["g"]))
        except socket.timeout:
            pass
        except (ValueError, TypeError, KeyError):
            continue

        while giro and giro[0][0] < ahora - VENTANA_VIBRACION:
            giro.popleft()

        if ahora - ultimo_dato > 2.0:
            giro.clear()
            with candado:
                estado["imu"] = None
            continue

        if ahora - ultimo_calculo >= 0.5:
            ultimo_calculo = ahora
            # Frecuencia de muestreo real: muestras llegadas después del primer paquete
            # / tiempo transcurrido desde ese paquete (las del primero son anteriores).
            duracion = giro[-1][0] - giro[0][0] if len(giro) > 1 else 0.0
            nuevas = sum(len(m) for _, m in giro) - len(giro[0][1]) if giro else 0
            fs = nuevas / duracion if duracion > 0 else 0.0
            if duracion >= VENTANA_VIBRACION * 0.5 and fs > 2 * FREC_MIN_VIBRACION:
                vibracion = analizar_vibracion([g for _, m in giro for g in m], fs)
                vibracion["muestreo"] = round(fs)
                vibracion["max_medible"] = round(fs / 2)
            else:
                vibracion = {"calibrando": True}
            with candado:
                estado["imu"] = {
                    "roll": angulos[0] if angulos else None,
                    "pitch": angulos[1] if angulos else None,
                    "yaw": angulos[2] if angulos else None,
                    **vibracion,
                }


def leer_camara():
    """Lee la cámara sin parar y guarda solo la imagen más reciente, sin decodificarla:
    el vídeo la reenvía tal cual y la detección decodifica solo la que va a analizar."""
    global ultimo_jpg, hora_captura, id_frame
    buffer = bytearray()
    while activo:
        datos = camara.stdout.read(4096)
        if not datos:
            break
        buffer.extend(datos)
        while True:
            inicio = buffer.find(b"\xff\xd8")
            if inicio == -1:
                buffer.clear()
                break
            fin = buffer.find(b"\xff\xd9", inicio + 2)
            if fin == -1:
                del buffer[:inicio]
                break
            jpg = bytes(buffer[inicio:fin + 2])
            del buffer[:fin + 2]
            with candado:
                ultimo_jpg = jpg
                hora_captura = time.time()
                id_frame += 1
                nuevo_frame.notify_all()


def decodificar(jpg):
    return cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)


def proceso_detector(conexion):
    """Proceso aparte: recibe JPEG, ejecuta el modelo, guarda las fotos de alerta y
    devuelve solo lo necesario para dibujar (cajas y contornos) y para el estado."""
    try:
        os.sched_setaffinity(0, NUCLEOS_DETECTOR)
    except (AttributeError, OSError):
        pass
    os.nice(10)
    modelo = cargar_modelo()
    seguidas = 0
    ultima_captura = 0.0
    while True:
        try:
            jpg = conexion.recv()
        except (EOFError, OSError):
            return
        frame = decodificar(jpg)
        if frame is None:
            conexion.send(None)
            continue
        t = time.time()
        r = modelo.predict(frame, imgsz=TAMANO, conf=CONFIANZA, verbose=False)[0]
        n = len(r.boxes) if r.boxes is not None else 0
        conf_max = float(r.boxes.conf.max()) if n else 0.0
        seguidas = seguidas + 1 if n else 0
        alerta = seguidas >= DETECCIONES_SEGUIDAS

        nueva = None
        ahora = time.time()
        if alerta and ahora - ultima_captura >= ESPERA_CAPTURAS:
            ultima_captura = ahora
            nombre = datetime.now().strftime("grieta_%Y%m%d_%H%M%S.jpg")
            cv2.imwrite(str(CAPTURAS / nombre), r.plot())
            nueva = {
                "hora": datetime.now().strftime("%H:%M:%S"),
                "grietas": n,
                "confianza": round(conf_max, 2),
                "foto": nombre,
            }

        cajas = [(*map(int, caja), float(c)) for caja, c in
                 zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist())] if n else []
        contornos = [np.asarray(c, dtype=np.int32) for c in r.masks.xy if len(c)] \
            if n and r.masks is not None else []
        conexion.send({
            "grietas": n,
            "confianza_max": round(conf_max, 2),
            "alerta": alerta,
            "detecciones_por_seg": round(1.0 / max(time.time() - t, 1e-6), 1),
            "nueva": nueva,
            "cajas": cajas,
            "contornos": contornos,
        })


def detectar(conexion):
    """Envía al proceso detector la imagen más reciente cada vez que termina la anterior."""
    global ultimo_resultado, id_resultado
    visto = -1
    while activo:
        with candado:
            nuevo_frame.wait_for(lambda: id_frame != visto or not activo, timeout=1.0)
            jpg, visto = ultimo_jpg, id_frame
        if jpg is None:
            continue
        try:
            conexion.send(jpg)
            resultado = conexion.recv()
        except (EOFError, OSError):
            print("El proceso de detección terminó; el vídeo sigue sin detección.")
            return
        if resultado is None:
            continue
        nueva = resultado.pop("nueva")
        with candado:
            ultimo_resultado = resultado
            id_resultado += 1
            if nueva:
                historial.appendleft(nueva)
                estado["total_alertas"] += 1
            estado.update(
                grietas=resultado["grietas"],
                confianza_max=resultado["confianza_max"],
                alerta=resultado["alerta"],
                detecciones_por_seg=resultado["detecciones_por_seg"],
            )


def dibujar_deteccion(frame, resultado):
    """Dibuja contornos (relleno semitransparente) y cajas con su confianza."""
    if resultado["contornos"]:
        capa = frame.copy()
        cv2.fillPoly(capa, resultado["contornos"], (0, 0, 255))
        frame = cv2.addWeighted(capa, 0.4, frame, 0.6, 0)
    for x1, y1, x2, y2, conf in resultado["cajas"]:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.putText(frame, f"grieta {conf:.2f}", (x1, max(14, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return frame


def imagen_actual():
    """Devuelve (JPEG, hora de captura) de la imagen más reciente.

    Sin grietas se reenvía el JPEG de la cámara tal cual, que es lo más rápido; con
    grietas se dibuja la última detección encima. Solo se prepara cuando el navegador
    la pide, así no se gasta CPU si nadie mira."""
    global cache_web
    with candado:
        jpg, captura, clave, r = ultimo_jpg, hora_captura, (id_frame, id_resultado), ultimo_resultado
        if cache_web[0] == clave:
            return cache_web[1], captura
    if jpg is None:
        return None, captura
    if r is None or not r["cajas"]:
        return jpg, captura
    frame = decodificar(jpg)
    if frame is None:
        return None, captura
    imagen = dibujar_deteccion(frame, r)
    if imagen.shape[1] != ANCHO_WEB:
        alto = int(imagen.shape[0] * ANCHO_WEB / imagen.shape[1])
        imagen = cv2.resize(imagen, (ANCHO_WEB, alto), interpolation=cv2.INTER_AREA)
    ok, codificada = cv2.imencode(".jpg", imagen, [cv2.IMWRITE_JPEG_QUALITY, CALIDAD_JPEG])
    if not ok:
        return None, captura
    datos = codificada.tobytes()
    with candado:
        cache_web = (clave, datos)
    return datos, captura


# --- Servidor web ---
import logging
logging.getLogger("werkzeug").setLevel(logging.ERROR)  # no llenar la terminal con cada petición
app = Flask(__name__)

PAGINA = """<!doctype html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Detector de grietas</title>
<style>
  :root { --fondo:#111418; --panel:#1b2027; --texto:#e8ecf1; --suave:#9aa5b1;
          --ok:#2fbf71; --alerta:#e5484d; --borde:#2a313a; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--fondo); color:var(--texto);
         font:15px/1.4 system-ui, -apple-system, "Segoe UI", sans-serif; }
  header { display:flex; align-items:center; justify-content:space-between;
           padding:12px 16px; border-bottom:1px solid var(--borde); gap:12px; flex-wrap:wrap; }
  h1 { font-size:18px; margin:0; }
  button { background:var(--panel); color:var(--texto); border:1px solid var(--borde);
           border-radius:8px; padding:8px 14px; font:inherit; cursor:pointer; }
  button.on { border-color:var(--ok); color:var(--ok); }
  main { display:grid; grid-template-columns:minmax(0,1fr) 320px; gap:16px; padding:16px; }
  @media (max-width:900px) { main { grid-template-columns:1fr; } }
  .video { position:relative; align-self:start; background:#000; border-radius:10px;
           overflow:hidden; border:3px solid var(--borde); }
  .video canvas { display:block; width:100%; height:auto; }
  .hud { position:absolute; top:8px; left:8px; display:flex; gap:6px; flex-wrap:wrap; }
  .hud span { background:rgba(0,0,0,.65); color:#fff; border-radius:6px; padding:3px 8px;
              font:600 13px/1.3 ui-monospace, "SFMono-Regular", Menlo, monospace;
              font-variant-numeric:tabular-nums; }
  .hud span.ok { color:#5ee39a; } .hud span.medio { color:#f5c451; } .hud span.mal { color:#ff7b7f; }
  .panel { background:var(--panel); border:1px solid var(--borde); border-radius:10px; padding:14px; }
  .lado { display:flex; flex-direction:column; gap:16px; }
  #aviso { font-size:20px; font-weight:700; text-align:center; padding:18px; border-radius:10px;
           background:#16351f; color:var(--ok); border:1px solid #1f5c33; }
  body.alerta #aviso { background:var(--alerta); color:#fff; border-color:var(--alerta);
                       animation:parpadeo 0.8s infinite alternate; }
  body.alerta .video { border-color:var(--alerta); }
  body.sinconexion #aviso { background:#3a3320; color:#f5c451; border-color:#5c4d1f; }
  @keyframes parpadeo { from { opacity:1; } to { opacity:0.65; } }
  .datos { display:grid; grid-template-columns:1fr 1fr; gap:10px; }
  .dato span { display:block; color:var(--suave); font-size:12px; }
  .dato b { font-size:20px; font-variant-numeric:tabular-nums; }
  h2 { font-size:14px; color:var(--suave); margin:0 0 10px; font-weight:600; }
  ul { list-style:none; margin:0; padding:0; max-height:340px; overflow:auto; }
  li { display:flex; gap:10px; align-items:center; padding:6px 0; border-top:1px solid var(--borde); }
  li:first-child { border-top:0; }
  li img { width:72px; height:54px; object-fit:cover; border-radius:4px; }
  li a { color:inherit; text-decoration:none; display:flex; gap:10px; align-items:center; }
  .vacio { color:var(--suave); }
  .datos.tres { grid-template-columns:1fr 1fr 1fr; }
  .dato small { color:var(--suave); font-size:12px; font-weight:400; }
  .nota { color:var(--suave); font-size:12px; margin:10px 0 0; }
  .imu-titulo { display:flex; justify-content:space-between; align-items:baseline; }
</style>
</head>
<body>
<header>
  <h1>Detector de grietas · Raspberry</h1>
  <button id="sonido">🔇 Activar sonido de alerta</button>
</header>
<main>
  <div class="video">
    <canvas id="camara" width="640" height="480" aria-label="Cámara en vivo"></canvas>
    <div class="hud" title="Latencia: desde que la imagen sale de la cámara hasta que se dibuja aquí (no incluye la exposición del sensor, hasta 33 ms). Meta: menos de 150 ms.">
      <span id="fps">– fps</span><span id="lat">latencia –</span><span id="rtt">Wi-Fi –</span>
    </div>
  </div>
  <div class="lado">
    <div id="aviso">Conectando…</div>
    <div class="panel datos">
      <div class="dato"><span>Grietas en imagen</span><b id="grietas">–</b></div>
      <div class="dato"><span>Confianza máx.</span><b id="conf">–</b></div>
      <div class="dato"><span>Detecciones/s</span><b id="dps">–</b></div>
      <div class="dato"><span>Alertas totales</span><b id="total">–</b></div>
    </div>
    <div class="panel">
      <div class="imu-titulo"><h2>Datos del IMU</h2><span class="nota" id="imu-estado" style="margin:0">Sin datos</span></div>
      <div class="datos tres">
        <div class="dato"><span>Roll (X)</span><b id="roll">–</b></div>
        <div class="dato"><span>Pitch (Y)</span><b id="pitch">–</b></div>
        <div class="dato"><span>Yaw (Z)</span><b id="yaw">–</b></div>
      </div>
      <div class="datos" style="margin-top:12px">
        <div class="dato"><span>Frecuencia de vibración</span><b id="vfrec">–</b></div>
        <div class="dato"><span>Intensidad (vel. angular RMS)</span><b id="vint">–</b></div>
        <div class="dato"><span>Amplitud de oscilación</span><b id="vamp">–</b></div>
        <div class="dato"><span>Eje dominante</span><b id="veje">–</b></div>
      </div>
      <p class="nota" id="imu-nota"></p>
    </div>
    <div class="panel">
      <h2>Últimas grietas detectadas</h2>
      <ul id="historial"><li class="vacio">Sin detecciones todavía</li></ul>
    </div>
  </div>
</main>
<script>
  let sonido = false, audio = null, ultimoBeep = 0;
  const boton = document.getElementById('sonido');
  boton.onclick = () => {
    sonido = !sonido;
    if (sonido && !audio) audio = new (window.AudioContext || window.webkitAudioContext)();
    boton.textContent = sonido ? '🔊 Sonido activado' : '🔇 Activar sonido de alerta';
    boton.classList.toggle('on', sonido);
  };
  function beep() {
    if (!sonido || !audio) return;
    const o = audio.createOscillator(), g = audio.createGain();
    o.frequency.value = 880; o.connect(g); g.connect(audio.destination);
    g.gain.setValueAtTime(0.3, audio.currentTime);
    g.gain.exponentialRampToValueAtTime(0.001, audio.currentTime + 0.4);
    o.start(); o.stop(audio.currentTime + 0.4);
  }
  let historialPrevio = '';
  function mostrar(e) {
      document.body.classList.remove('sinconexion');
      document.body.classList.toggle('alerta', e.alerta);
      document.getElementById('aviso').textContent = e.alerta ? '⚠ ¡GRIETA DETECTADA!' : 'Sin grietas';
      document.title = e.alerta ? '⚠ GRIETA · Detector' : 'Detector de grietas';
      document.getElementById('grietas').textContent = e.grietas;
      document.getElementById('conf').textContent = e.confianza_max.toFixed(2);
      document.getElementById('dps').textContent = e.detecciones_por_seg.toFixed(1);
      document.getElementById('total').textContent = e.total_alertas;
      if (e.alerta && Date.now() - ultimoBeep > 1500) { beep(); ultimoBeep = Date.now(); }
      mostrarImu(e.imu);
      // Las fotos del historial solo se vuelven a pedir si cambió la lista
      const clave = e.historial.map(h => h.foto).join(',');
      if (clave !== historialPrevio) {
        historialPrevio = clave;
        document.getElementById('historial').innerHTML = e.historial.length ? e.historial.slice(0, 8).map(h =>
          `<li><a href="/capturas/${h.foto}" target="_blank"><img src="/capturas/${h.foto}" alt="" loading="lazy">
           <div><b>${h.hora}</b><br>${h.grietas} grieta(s) · conf. ${h.confianza.toFixed(2)}</div></a></li>`
        ).join('') : '<li class="vacio">Sin detecciones todavía</li>';
      }
  }
  function mostrarImu(m) {
    const fija = (id, txt) => document.getElementById(id).innerHTML = txt;
    const grados = v => v === null || v === undefined ? '–' : v.toFixed(1) + '<small>°</small>';
    if (!m) {
      ['roll', 'pitch', 'yaw', 'vfrec', 'vint', 'vamp', 'veje'].forEach(id => fija(id, '–'));
      fija('imu-estado', 'Sin datos');
      fija('imu-nota', 'Inicia el emisor del BNO085 (start_gecko_pc.sh en el PC).');
      return;
    }
    fija('roll', grados(m.roll)); fija('pitch', grados(m.pitch)); fija('yaw', grados(m.yaw));
    if (m.calibrando) {
      ['vfrec', 'vint', 'vamp', 'veje'].forEach(id => fija(id, '–'));
      fija('imu-estado', 'Midiendo…');
      fija('imu-nota', 'Reuniendo muestras del giroscopio…');
      return;
    }
    fija('imu-estado', 'En vivo');
    fija('vfrec', m.frecuencia === null ? 'Sin vibración' : m.frecuencia.toFixed(1) + ' <small>Hz</small>');
    fija('vint', m.intensidad.toFixed(1) + ' <small>°/s</small>');
    fija('vamp', m.amplitud === null ? '–' : '±' + m.amplitud.toFixed(2) + '<small>°</small>');
    fija('veje', m.eje);
    fija('imu-nota', `Giroscopio a ${m.muestreo} Hz · mide vibraciones hasta ${m.max_medible} Hz`);
  }
  function sinConexion() {
    document.body.classList.add('sinconexion');
    document.body.classList.remove('alerta');
    document.getElementById('aviso').textContent = 'Sin conexión con la Raspberry';
  }

  // --- Vídeo con FPS y latencia ---
  // El vídeo se lee con fetch (no con <img>) para conocer la hora de captura de cada
  // imagen (cabecera X-Captura, reloj de la Raspberry). El reloj del navegador se
  // sincroniza con /reloj usando la petición de menor ida y vuelta.
  const cam = document.getElementById('camara'), ctx = cam.getContext('2d');
  const META_MS = 150;
  let desfase = null, rttWifi = null;      // desfase = reloj Raspberry - reloj navegador
  let dibujos = [], latencias = [];
  let pendiente = null, dibujando = false, videoAbierto = null;

  async function sincronizar() {
    let mejor = null;
    for (let i = 0; i < 5; i++) {
      const t0 = Date.now();
      try {
        const { t } = await (await fetch('/reloj', { cache: 'no-store' })).json();
        const t1 = Date.now();
        if (!mejor || t1 - t0 < mejor.rtt) mejor = { rtt: t1 - t0, desfase: t - (t0 + t1) / 2 };
      } catch (e) { return; }
    }
    desfase = mejor.desfase; rttWifi = mejor.rtt;
  }

  function buscarFinCabecera(b, desde) {
    for (let i = desde; i + 3 < b.length; i++)
      if (b[i] === 13 && b[i + 1] === 10 && b[i + 2] === 13 && b[i + 3] === 10) return i;
    return -1;
  }

  async function leerVideo(senal) {
    const resp = await fetch('/video?t=' + Date.now(), { signal: senal, cache: 'no-store' });
    const lector = resp.body.getReader(), texto = new TextDecoder();
    let buf = new Uint8Array(0);
    for (;;) {
      const { value, done } = await lector.read();
      if (done) throw new Error('fin del vídeo');
      const unido = new Uint8Array(buf.length + value.length);
      unido.set(buf); unido.set(value, buf.length); buf = unido;
      for (;;) {
        const fin = buscarFinCabecera(buf, 0);
        if (fin < 0) break;
        const cab = texto.decode(buf.subarray(0, fin));
        const largo = parseInt((/Content-Length:\\s*(\\d+)/i.exec(cab) || [])[1]);
        const captura = parseFloat((/X-Captura:\\s*([\\d.]+)/i.exec(cab) || [])[1]);
        if (!(largo >= 0)) { buf = buf.slice(fin + 4); continue; }
        if (buf.length < fin + 4 + largo) break;
        mostrarCuadro(buf.slice(fin + 4, fin + 4 + largo), captura);
        buf = buf.slice(fin + 4 + largo);
      }
    }
  }

  // Si el navegador va más lento que la cámara, se dibuja solo la imagen más nueva.
  function mostrarCuadro(jpg, captura) {
    pendiente = { jpg, captura };
    if (!dibujando) dibujarSiguiente();
  }
  async function dibujarSiguiente() {
    dibujando = true;
    while (pendiente) {
      const { jpg, captura } = pendiente; pendiente = null;
      try {
        const bmp = await createImageBitmap(new Blob([jpg], { type: 'image/jpeg' }));
        if (cam.width !== bmp.width || cam.height !== bmp.height) {
          cam.width = bmp.width; cam.height = bmp.height;
        }
        ctx.drawImage(bmp, 0, 0); bmp.close();
        const ahora = Date.now();
        dibujos.push(ahora);
        if (desfase !== null && captura > 0) latencias.push(ahora - (captura - desfase));
      } catch (e) { /* imagen corrupta: se salta */ }
    }
    dibujando = false;
  }

  function actualizarHud() {
    const ahora = Date.now();
    dibujos = dibujos.filter(t => ahora - t <= 1000);
    latencias = latencias.slice(-30);
    document.getElementById('fps').textContent = videoAbierto ? `${dibujos.length} fps` : '– fps';
    const lat = document.getElementById('lat');
    if (latencias.length && dibujos.length) {
      const orden = [...latencias].sort((a, b) => a - b);
      const mediana = Math.max(0, Math.round(orden[orden.length >> 1]));
      lat.textContent = `latencia ${mediana} ms`;
      lat.className = mediana < META_MS ? 'ok' : mediana < 2 * META_MS ? 'medio' : 'mal';
    } else { lat.textContent = 'latencia –'; lat.className = ''; }
    document.getElementById('rtt').textContent = rttWifi === null ? 'Wi-Fi –' : `Wi-Fi ${Math.round(rttWifi)} ms`;
  }
  setInterval(actualizarHud, 500);
  setInterval(() => { if (videoAbierto) sincronizar(); }, 10000);

  async function abrirVideo() {
    while (videoAbierto) {
      const control = videoAbierto;
      try { await sincronizar(); await leerVideo(control.signal); }
      catch (e) { if (control.signal.aborted) return; }
      await new Promise(r => setTimeout(r, 1000));
    }
  }

  // Estado y vídeo por conexiones abiertas; se pausan si la pestaña no está visible
  // para no gastar datos ni peticiones de ngrok.
  let fuente = null;
  function conectar() {
    if (fuente) return;
    fuente = new EventSource('/eventos');
    fuente.onmessage = ev => mostrar(JSON.parse(ev.data));
    fuente.onerror = sinConexion;
    videoAbierto = new AbortController();
    abrirVideo();
  }
  function pausar() {
    if (fuente) { fuente.close(); fuente = null; }
    if (videoAbierto) { videoAbierto.abort(); videoAbierto = null; }
    latencias = []; dibujos = [];
  }
  document.addEventListener('visibilitychange', () => document.hidden ? pausar() : conectar());
  conectar();
</script>
</body>
</html>"""


@app.route("/")
def inicio():
    return PAGINA


@app.route("/estado")
def ver_estado():
    with candado:
        return jsonify(dict(estado, historial=list(historial)))


@app.route("/video")
def video():
    """Flujo de vídeo que envía cada imagen apenas llega de la cámara.

    Siempre manda la más reciente (si el navegador va lento se saltan imágenes en vez
    de acumular retraso). Cada imagen lleva su hora de captura en X-Captura (ms desde
    1970, reloj de la Raspberry) para que la página calcule la latencia."""
    def generar():
        visto = -1
        while activo:
            with candado:
                nuevo_frame.wait_for(lambda: id_frame != visto or not activo, timeout=1.0)
                visto = id_frame
            datos, captura = imagen_actual()
            if datos is None:
                continue
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                   + str(len(datos)).encode()
                   + b"\r\nX-Captura: " + f"{captura * 1000:.1f}".encode()
                   + b"\r\n\r\n" + datos + b"\r\n")

    respuesta = Response(generar(), mimetype="multipart/x-mixed-replace; boundary=frame")
    respuesta.headers["Cache-Control"] = "no-store"
    respuesta.headers["X-Accel-Buffering"] = "no"
    return respuesta


@app.route("/reloj")
def reloj():
    """Hora de la Raspberry en ms, para que la página sincronice su reloj."""
    respuesta = jsonify(t=time.time() * 1000)
    respuesta.headers["Cache-Control"] = "no-store"
    return respuesta


@app.route("/eventos")
def eventos():
    """Envía el estado al navegador por una sola conexión abierta (ahorra peticiones)."""
    import json

    def generar():
        while activo:
            with candado:
                datos = json.dumps(dict(estado, historial=list(historial)))
            yield f"data: {datos}\n\n"
            time.sleep(0.5)

    respuesta = Response(generar(), mimetype="text/event-stream")
    respuesta.headers["Cache-Control"] = "no-store"
    respuesta.headers["X-Accel-Buffering"] = "no"
    return respuesta


@app.route("/capturas/<nombre>")
def capturas(nombre):
    if "/" in nombre or not nombre.endswith(".jpg"):
        abort(404)
    return send_from_directory(CAPTURAS, nombre)


def ip_local():
    # 10.42.0.1 es el PC cuando la Raspberry está en su hotspot (sin internet)
    for destino in ("8.8.8.8", "10.42.0.1"):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect((destino, 80))
                return s.getsockname()[0]
        except OSError:
            continue
    return "IP-de-la-Raspberry"


if __name__ == "__main__":
    # El proceso detector se crea antes que la cámara y los hilos para no heredarlos.
    conexion, conexion_detector = multiprocessing.Pipe()
    detector = multiprocessing.get_context("fork").Process(
        target=proceso_detector, args=(conexion_detector,), daemon=True)
    detector.start()
    conexion_detector.close()
    camara = iniciar_camara()
    threading.Thread(target=leer_camara, daemon=True).start()
    threading.Thread(target=detectar, args=(conexion,), daemon=True).start()
    threading.Thread(target=leer_imu, daemon=True).start()
    print(f"\nAbre en tu computador:  http://{ip_local()}:{PUERTO}\n(Ctrl+C para detener)\n")
    try:
        app.run(host="0.0.0.0", port=PUERTO, threaded=True)
    finally:
        activo = False
        camara.terminate()
        camara.wait()
        detector.terminate()
        cv2.destroyAllWindows()
