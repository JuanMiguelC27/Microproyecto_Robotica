#!/usr/bin/env bash
# Runs ON THE RASPBERRY, sent by gecko_pi.sh through `ssh ... bash -s`.
# Stops the BNO085 sender with Ctrl+C so it closes the socket and releases I2C.

set -u

pattern='^[^ ]*python[^ ]* .*[b]no085_sender\.py'

# Si el emisor corre como servicio de usuario (gecko-imu.service), se para antes
# con systemctl, para que systemd no lo relance y compita por el I2C.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
if command -v systemctl >/dev/null 2>&1; then
    systemctl --user stop gecko-imu.service >/dev/null 2>&1 || true
fi

if ! pgrep -f "$pattern" >/dev/null; then
    echo "not running"
    exit 0
fi

pkill -INT -f "$pattern"
for _ in $(seq 1 20); do
    if ! pgrep -f "$pattern" >/dev/null; then
        echo "stopped"
        exit 0
    fi
    sleep 0.25
done

pkill -KILL -f "$pattern" 2>/dev/null || true
echo "killed"
