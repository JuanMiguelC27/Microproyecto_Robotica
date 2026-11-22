#!/usr/bin/env bash
# CLI para el puente ESP32. Uso: esp32ctl.sh start|stop|status|send|line|tail|run|runl
D="${GECKO_SERIAL_DIR:-$HOME/gecko_serial}"
HERE="$(cd "$(dirname "$0")" && pwd)"
PIDF="$D/bridge.pid"; LOG="$D/esp32.log"; FIFO="$D/cmd.fifo"
running() { [ -f "$PIDF" ] && kill -0 "$(cat "$PIDF")" 2>/dev/null; }
need() { running || { echo "ERROR: el bridge no esta corriendo (usa: $0 start)" >&2; exit 1; }; }
managed() {
  [ "$D" = "$HOME/gecko_serial" ] && systemctl --user is-enabled --quiet gecko-serial.service
}
record_stop() {
  case "$1" in
    x|X|' ')
      if [ "$D" = "$HOME/gecko_serial" ] && [ -n "${XDG_RUNTIME_DIR:-}" ]; then
        touch "$XDG_RUNTIME_DIR/gecko-paro-manual"
      fi ;;
  esac
}
cmd="$1"; shift
case "$cmd" in
  start)
    if running; then echo "ya corriendo (pid $(cat "$PIDF"))"; exit 0; fi
    if managed; then systemctl --user start gecko-serial.service; exit $?; fi
    mkdir -p "$D"
    setsid nohup /usr/bin/python3 "$HERE/esp32_bridge.py" --dir "$D" ${ESP32_PORT:+--port "$ESP32_PORT"} \
      >"$D/bridge.out" 2>&1 < /dev/null &
    sleep 1
    running && echo "iniciado (pid $(cat "$PIDF"))" || { echo "ERROR: no arranco:" >&2; cat "$D/bridge.out" >&2; exit 1; } ;;
  stop)
    record_stop x
    if [ "$D" = "$HOME/gecko_serial" ] && systemctl --user is-active --quiet gecko-serial.service; then
      systemctl --user stop gecko-serial.service && echo "detenido"
      exit $?
    fi
    need; p=$(cat "$PIDF"); kill "$p"
    for _ in $(seq 20); do kill -0 "$p" 2>/dev/null || break; sleep 0.1; done
    echo "detenido" ;;
  status)
    if running; then echo "corriendo (pid $(cat "$PIDF")), log $(stat -c %s "$LOG") bytes"; else echo "detenido"; exit 1; fi ;;
  send) record_stop "$1"; need; printf '%s' "$1" > "$FIFO" ;;
  line) record_stop "$1"; need; printf '%s\n' "$1" > "$FIFO" ;;
  tail) need; tail -n "${1:-60}" "$LOG" ;;
  run|runl)
    record_stop "$1"
    need; n=$(stat -c %s "$LOG")
    if [ "$cmd" = run ]; then printf '%s' "$1" > "$FIFO"; else printf '%s\n' "$1" > "$FIFO"; fi
    sleep "${2:-2}"; tail -c +$((n+1)) "$LOG" ;;
  *) echo "uso: $0 start|stop|status|send <t>|line <t>|tail [N]|run <t> [s]|runl <t> [s]" >&2; exit 2 ;;
esac
