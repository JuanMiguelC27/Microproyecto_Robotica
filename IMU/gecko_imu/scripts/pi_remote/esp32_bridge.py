#!/usr/bin/env python3
"""Puente serial ESP32 <-> FIFO/log. Mantiene el puerto abierto entre comandos SSH."""
import argparse, datetime, fcntl, os, signal, threading, time
import serial

ap = argparse.ArgumentParser()
ap.add_argument("--port", default="/dev/ttyUSB0")
ap.add_argument("--baud", type=int, default=115200)
ap.add_argument("--dir", default="~/gecko_serial")
a = ap.parse_args()
D = os.path.expanduser(a.dir)
os.makedirs(D, exist_ok=True)
# Conserva el propietario único del FIFO, el PID y el puerto durante todo el proceso.
owner = open(os.path.join(D, "bridge.lock"), "a")
try:
    fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    raise SystemExit("El directorio serial ya está a cargo de otro puente.")
LOG, FIFO, PID = (os.path.join(D, n) for n in ("esp32.log", "cmd.fifo", "bridge.pid"))
if not os.path.exists(FIFO):
    os.mkfifo(FIFO)

logf = open(LOG, "a", buffering=1, encoding="utf-8", errors="replace")
lock = threading.Lock()
ser = serial.Serial()
stop = threading.Event()


def log(s):
    with lock:
        logf.write(s)
        logf.flush()


def mark(s):
    log("\n=== %s %s ===\n" % (s, datetime.datetime.now().isoformat(timespec="seconds")))


def open_port():
    ser.port, ser.baudrate, ser.timeout = a.port, a.baud, 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()


def reader():
    dec = __import__("codecs").getincrementaldecoder("utf-8")(errors="replace")
    while not stop.is_set():
        try:
            if not ser.is_open:
                open_port()
                mark("port open " + a.port)
            data = ser.read(ser.in_waiting or 1)
            if data:
                log(dec.decode(data))
        except (serial.SerialException, OSError) as e:
            mark("port error %r, retry in 2s" % e)
            try:
                ser.close()
            except Exception:
                pass
            stop.wait(2)


def write(b):
    try:
        if ser.is_open:
            ser.write(b)
            ser.flush()
    except (serial.SerialException, OSError) as e:
        log("\n=== write error %r ===\n" % e)


def shutdown(*_):
    stop.set()


signal.signal(signal.SIGTERM, shutdown)
signal.signal(signal.SIGINT, shutdown)
with open(PID, "w") as f:
    f.write(str(os.getpid()))
mark("bridge start")
threading.Thread(target=reader, daemon=True).start()

# O_RDWR: el propio proceso es escritor, nunca hay EOF continuo.
fd = os.open(FIFO, os.O_RDWR | os.O_NONBLOCK)
import select
while not stop.is_set():
    if select.select([fd], [], [], 0.2)[0]:
        try:
            b = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if b:
            write(b)

write(b"x")
time.sleep(0.2)
mark("bridge stop")
try:
    ser.close()
finally:
    try:
        os.remove(PID)
    except OSError:
        pass
