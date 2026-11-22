#!/usr/bin/env bash
# Runs ON THE RASPBERRY, sent by gecko_pi.sh through `ssh ... bash -s`.
# Replaces any running BNO085 sender with a new one in the background.
#
# Usage: start_sender.sh PYTHON SENDER [SENDER_ARGS...]

set -u

python_bin="$1"
sender="$2"
shift 2

log="${GECKO_SENDER_LOG:-$HOME/gecko_imu_sender.log}"

# Anchored to the interpreter so this shell, whose arguments also name the
# sender, is never matched.
pattern='^[^ ]*python[^ ]* .*[b]no085_sender\.py'

[[ -x "$python_bin" ]] || { echo "No existe el intérprete $python_bin"; exit 2; }
[[ -f "$sender" ]] || { echo "No existe el emisor $sender"; exit 2; }

# Si el emisor corre como servicio de usuario (gecko-imu.service), se para antes
# con systemctl, para que systemd no lo relance y compita por el I2C.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if command -v systemctl >/dev/null 2>&1; then
    systemctl --user stop gecko-imu.service >/dev/null 2>&1 || true
fi

# Two senders would compete for the I2C bus. Ctrl+C lets the old one release it.
if pgrep -f "$pattern" >/dev/null; then
    pkill -INT -f "$pattern"
    for _ in $(seq 1 20); do
        pgrep -f "$pattern" >/dev/null || break
        sleep 0.25
    done
    pkill -KILL -f "$pattern" 2>/dev/null || true
fi

# Background jobs of a script start with Ctrl+C ignored, so Python would never
# raise KeyboardInterrupt and could not release I2C. Restore it before running.
restore_ctrl_c='import runpy, signal, sys
signal.signal(signal.SIGINT, signal.default_int_handler)
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name="__main__")'

nohup "$python_bin" -u -c "$restore_ctrl_c" "$sender" "$@" >"$log" 2>&1 </dev/null &
sleep 3

if pgrep -f "$pattern" >/dev/null; then
    echo "running"
    tail -n 3 "$log"
else
    echo "stopped"
    tail -n 20 "$log"
    exit 1
fi
