#!/usr/bin/env bash
# Inicia o detiene por SSH el emisor de la IMU que corre en la Raspberry.
#
# Uso:
#   gecko_pi.sh start   Espera a que la Raspberry entre al hotspot e inicia el emisor.
#   gecko_pi.sh stop    Detiene el emisor con Ctrl+C.
#
# Lo llama start_gecko_pc.sh. Requiere que la clave SSH de este PC esté
# autorizada en la Raspberry.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PI_USER="${PI_USER:-jarvis}"
PI_HOSTNAME="${PI_HOSTNAME:-jarvis-desktop}"
PI_HOST="${PI_HOST:-$PI_HOSTNAME.local}"
# Rutas en la Raspberry; el ~ lo expande la Raspberry.
PI_PROJECT="${PI_PROJECT:-~/mini_sumo_imu}"
PI_PYTHON="${PI_PYTHON:-$PI_PROJECT/.venv/bin/python}"
PI_SENDER="${PI_SENDER:-$PI_PROJECT/scripts/bno085_sender.py}"

PC_IP="${PC_IP:-10.42.0.1}"
UDP_PORT="${UDP_PORT:-5005}"
SEND_RATE="${SEND_RATE:-30}"
PI_WAIT_S="${PI_WAIT_S:-120}"
WIFI_IFACE="${WIFI_IFACE:-wlo1}"
STATE_DIR="${STATE_DIR:-$HOME/.local/state/gecko_imu}"
HOST_CACHE="$STATE_DIR/pi_host"

SSH_OPTS=(
    -o BatchMode=yes
    -o ConnectTimeout=3
    -o StrictHostKeyChecking=accept-new
    -o ServerAliveInterval=5
    -o ServerAliveCountMax=3
)

say() {
    printf '[Raspberry] %s\n' "$*"
}

remote() {
    local host="$1"
    shift
    ssh "${SSH_OPTS[@]}" "$PI_USER@$host" "$@"
}

# Direcciones donde puede estar la Raspberry dentro del hotspot.
candidate_hosts() {
    local leases="/var/lib/NetworkManager/dnsmasq-$WIFI_IFACE.leases"

    printf '%s\n' "$PI_HOST"
    [[ -f "$HOST_CACHE" ]] && cat "$HOST_CACHE"
    [[ -f "$leases" ]] && awk '{print $3}' "$leases"
    ip -4 neigh show dev "$WIFI_IFACE" 2>/dev/null |
        awk '$1 ~ /^10\.42\.0\./ && $NF != "FAILED" {print $1}'
}

# Devuelve la primera dirección que acepta la clave y responde con el nombre esperado.
find_pi() {
    local host name

    while read -r host; do
        [[ -n "$host" ]] || continue
        name="$(remote "$host" hostname 2>/dev/null)" || continue
        if [[ "$name" == "$PI_HOSTNAME" ]]; then
            printf '%s\n' "$host"
            return 0
        fi
    done < <(candidate_hosts | awk '!seen[$0]++')

    return 1
}

start_sender() {
    local host="" deadline=$((SECONDS + PI_WAIT_S)) output

    say "Buscando $PI_USER@$PI_HOSTNAME (hasta $PI_WAIT_S s)..."

    while (( SECONDS < deadline )); do
        if host="$(find_pi)"; then
            break
        fi
        host=""
        sleep 3
    done

    if [[ -z "$host" ]]; then
        say "No se encontró la Raspberry. Comprueba que esté encendida y conectada a la red del hotspot."
        say "Si ya estaba encendida en otra red, reiníciala para que entre al hotspot."
        say "Después ejecuta: start_gecko_pc.sh pi"
        return 1
    fi

    # El nombre .local deja de resolverse por momentos; con la IP, stop sigue
    # encontrando la Raspberry. SSH_CONNECTION termina en "IP_servidor puerto".
    local ip
    ip="$(remote "$host" 'set -- $SSH_CONNECTION; echo "$3"' 2>/dev/null)"
    [[ "$ip" =~ ^[0-9]+(\.[0-9]+){3}$ ]] && host="$ip"

    mkdir -p "$STATE_DIR"
    printf '%s\n' "$host" > "$HOST_CACHE"
    say "Raspberry encontrada en $host. Iniciando el emisor de la IMU..."

    output="$(remote "$host" bash -s -- "$PI_PYTHON" "$PI_SENDER" \
        --host "$PC_IP" --port "$UDP_PORT" --rate "$SEND_RATE" \
        < "$SCRIPT_DIR/pi_remote/start_sender.sh" 2>&1)"
    local code=$?
    printf '%s\n' "$output" | sed 's/^/[Raspberry]   /'

    if (( code != 0 )); then
        if grep -q 'Permission denied' <<<"$output"; then
            say "La Raspberry no tiene permiso para usar el I2C sin una sesión abierta."
            say "Solución (una sola vez): ssh -t $PI_USER@$host 'sudo usermod -aG i2c $PI_USER && sudo reboot'"
        fi
        say "El emisor no quedó en ejecución. Registro en la Raspberry: ~/gecko_imu_sender.log"
        return 1
    fi

    say "Emisor en ejecución. Registro en la Raspberry: ~/gecko_imu_sender.log"
}

stop_sender() {
    local host output

    while read -r host; do
        [[ -n "$host" ]] || continue
        if output="$(remote "$host" bash -s < "$SCRIPT_DIR/pi_remote/stop_sender.sh" 2>/dev/null)"; then
            say "Emisor de la IMU: $output"
            return 0
        fi
    done < <({ [[ -f "$HOST_CACHE" ]] && cat "$HOST_CACHE"; printf '%s\n' "$PI_HOST"; } |
             awk '!seen[$0]++')

    say "No se pudo contactar la Raspberry para detener el emisor."
    return 0
}

case "${1:-}" in
    start) start_sender ;;
    stop) stop_sender ;;
    *) echo "Uso: $0 start|stop" >&2; exit 2 ;;
esac
