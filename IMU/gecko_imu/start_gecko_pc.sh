#!/usr/bin/env bash
# Inicia el gemelo digital del Gecko: hotspot, un mundo Gazebo Fortress NUEVO
# (empty.sdf), el vehículo, el puente ROS-Gazebo, los nodos y RViz con el panel
# de control.
#
# Uso:
#   ./start_gecko_pc.sh
#   ./start_gecko_pc.sh stop
#   ./start_gecko_pc.sh rviz   Reabre RViz sin reiniciar el gemelo.
#   ./start_gecko_pc.sh pi     Vuelve a iniciar el emisor de la IMU en la Raspberry.
#
# Variables opcionales:
#   GECKO_SKIP_HOTSPOT=1   No toca el Wi-Fi.
#   GECKO_SKIP_PI=1        No inicia ni detiene el emisor de la Raspberry por SSH.
#                          Por defecto se omite cuando se omite el hotspot.
#   GECKO_HEADLESS=1       Ejecuta Gazebo y RViz sin abrir ventanas.

set -Eeuo pipefail

# --------------------------- CONFIGURACIÓN ---------------------------

WIFI_IFACE="${WIFI_IFACE:-wlo1}"
HOTSPOT_SSID="${HOTSPOT_SSID:-Gecko-IMU}"
HOTSPOT_PASSWORD="${HOTSPOT_PASSWORD:-Gecko2026!}"
HOTSPOT_IP="${HOTSPOT_IP:-10.42.0.1/24}"
# Canal de 2,4 GHz del hotspot. El 11 estaba casi libre en el campus; el 1, que elegía
# NetworkManager solo, lo comparten muchas redes y el vídeo de la cámara se atascaba.
HOTSPOT_CHANNEL="${HOTSPOT_CHANNEL:-11}"

GECKO_SKIP_HOTSPOT="${GECKO_SKIP_HOTSPOT:-0}"
GECKO_SKIP_PI="${GECKO_SKIP_PI:-$GECKO_SKIP_HOTSPOT}"
GECKO_HEADLESS="${GECKO_HEADLESS:-0}"

PROJECT_DIR="${PROJECT_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

SCRIPTS_DIR="$PROJECT_DIR/scripts"
WALL_SDF="$PROJECT_DIR/models/wall/wall.sdf"
MODELS_DIR="$PROJECT_DIR/models/gecko"
WS_DIR="$PROJECT_DIR/ros2_ws"
RVIZ_CONFIG="$PROJECT_DIR/rviz/gecko.rviz"

SOURCE_URDF="${SOURCE_URDF:-$PROJECT_DIR/../../Micro2_URDFV2/urdf/Micro2_URDFV2.urdf}"
ROBOT_URDF="$MODELS_DIR/gecko_gazebo.urdf"
RSP_PARAMS="$MODELS_DIR/robot_state_publisher.yaml"

ROS_SETUP="/opt/ros/humble/setup.bash"
SYSTEM_PYTHON="/usr/bin/python3"
BRIDGE_BIN="/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge"
RSP_BIN="/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher"

WORLD_NAME="empty"
ROBOT_NAME="gecko"
WALL_NAME="gecko_wall"

STATE_DIR="${STATE_DIR:-$HOME/.local/state/gecko_imu}"
LOG_DIR="$STATE_DIR/logs"
PID_DIR="$STATE_DIR/pids"

# gecko_pi.sh lee estas variables para encontrar la Raspberry y apuntarla al PC.
export WIFI_IFACE STATE_DIR
export PC_IP="${HOTSPOT_IP%/*}"


# ------------------------------ UTILIDADES ---------------------------

info() {
    printf '\n[PC] %s\n' "$*"
}

fail() {
    printf '\n[PC][ERROR] %s\n' "$*" >&2
    exit 1
}

require_command() {
    command -v "$1" >/dev/null 2>&1 ||
        fail "No se encontró el comando '$1'."
}


# ROS 2 Humble necesita el Python 3.10 del sistema, no el de Conda.
clean_conda_env() {

    local name

    PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -vi 'conda' | paste -sd ':' -)"
    export PATH

    while read -r name; do
        unset "$name"
    done < <(compgen -e | grep -E '^_?CONDA' || true)

    unset PYTHONHOME PYTHONPATH
}


# Iniciar un proceso en segundo plano y registrar su PID.
start_bg() {

    local name="$1"
    shift

    local pidfile="$PID_DIR/$name.pid"
    local logfile="$LOG_DIR/$name.log"

    if [[ -f "$pidfile" ]]; then

        local old_pid
        old_pid="$(cat "$pidfile" 2>/dev/null || true)"

        if [[ -n "$old_pid" ]] &&
           kill -0 "$old_pid" 2>/dev/null; then

            info "$name ya está ejecutándose (PID $old_pid)."
            return 0
        fi

        rm -f "$pidfile"
    fi

    nohup "$@" >>"$logfile" 2>&1 </dev/null &

    local new_pid=$!
    printf '%s\n' "$new_pid" > "$pidfile"

    sleep 1

    if ! kill -0 "$new_pid" 2>/dev/null; then

        rm -f "$pidfile"

        printf '\n--- Últimas líneas de %s ---\n' "$logfile" >&2
        tail -n 40 "$logfile" >&2 || true

        fail "No se pudo iniciar $name."
    fi

    info "$name iniciado (PID $new_pid)."
    info "Registro: $logfile"
}


# Esperar a que Gazebo publique un servicio.
wait_for_ign_service() {

    local service="$1"
    local attempts="${2:-60}"

    for ((i=1; i<=attempts; i++)); do

        if ign service -l 2>/dev/null | grep -F "$service" >/dev/null; then
            return 0
        fi

        sleep 1
    done

    return 1
}


# Acercar la cámara de Gazebo al vehículo, que mide unos 30 cm.
focus_gazebo_camera() {

    wait_for_ign_service "/gui/move_to" 30 || return 0

    ign service -s /gui/move_to \
        --reqtype ignition.msgs.StringMsg \
        --reptype ignition.msgs.Boolean \
        --timeout 3000 \
        --req "data: \"$ROBOT_NAME\"" \
        >/dev/null 2>&1 || true
}


# Eliminar una entidad del mundo Gazebo.
remove_entity_if_present() {

    local name="$1"

    ign service -s "/world/$WORLD_NAME/remove" \
        --reqtype ignition.msgs.Entity \
        --reptype ignition.msgs.Boolean \
        --timeout 3000 \
        --req "name: \"$name\", type: 2" \
        >/dev/null 2>&1 || true
}


# Cerrar TODAS las instancias anteriores de Gazebo.
stop_gazebo_processes() {

    if pgrep -f '[i]gn gazebo' >/dev/null 2>&1; then

        info "Cerrando las instancias anteriores de Gazebo..."

        pkill -TERM -f '[i]gn gazebo' 2>/dev/null || true

        for _ in {1..10}; do

            if ! pgrep -f '[i]gn gazebo' >/dev/null 2>&1; then
                break
            fi

            sleep 1
        done

        if pgrep -f '[i]gn gazebo' >/dev/null 2>&1; then

            info "Finalizando los procesos de Gazebo restantes..."

            pkill -KILL -f '[i]gn gazebo' 2>/dev/null || true

            sleep 1
        fi
    fi

    rm -f "$PID_DIR/gazebo.pid"
}


# Cerrar los nodos del sistema, incluidos los de una ejecución anterior.
# Los patrones exigen el ejecutable real para no cerrar editores ni otros proyectos.
stop_nodes() {

    local rviz='^[^ ]*[r]viz2 -d .*gecko\.rviz'
    local twin='^[^ ]*python3 .*[g]ecko_twin_node\.py'
    local bridge='^[^ ]*[p]arameter_bridge .*/world/empty'
    local pattern

    for pattern in \
        "$rviz" \
        '^[^ ]*[r]obot_state_publisher .*models/gecko/robot_state_publisher\.yaml' \
        '^[^ ]*python3 .*[w]all_climb_manager\.py' \
        '^[^ ]*python3 .*[i]mu_displacement_estimator\.py' \
        "$twin" \
        '^[^ ]*python3 .*[u]dp_imu_receiver\.py' \
        '^[^ ]*bash .*[g]ecko_pi\.sh start' \
        "$bridge"
    do
        pkill -TERM -f "$pattern" 2>/dev/null || true
    done

    for _ in {1..5}; do

        if ! pgrep -f "$rviz|$twin|$bridge" >/dev/null 2>&1; then
            break
        fi

        sleep 1
    done

    pkill -KILL -f "$rviz" 2>/dev/null || true

    find "$PID_DIR" -name '*.pid' ! -name 'gazebo.pid' -delete 2>/dev/null || true
}


# Compilar el panel de RViz solo si falta o cambió.
build_panel() {

    local stamp="$WS_DIR/install/.gecko_build_stamp"

    if [[ -f "$stamp" ]] &&
       [[ -z "$(find "$WS_DIR/src" -type f -newer "$stamp" -print -quit)" ]]; then
        return 0
    fi

    info "Compilando el panel de RViz..."

    (cd "$WS_DIR" && colcon build --packages-select gecko_rviz_panel) \
        >"$LOG_DIR/colcon.log" 2>&1 ||
        { tail -n 40 "$LOG_DIR/colcon.log" >&2; fail "No se pudo compilar el panel."; }

    touch "$stamp"
}


# Abrir RViz con el panel de control.
start_rviz() {

    if [[ "$GECKO_HEADLESS" == "1" ]]; then
        LIBGL_ALWAYS_SOFTWARE=1 start_bg rviz xvfb-run -a -s '-screen 0 1600x1000x24' \
            rviz2 -d "$RVIZ_CONFIG"
    else
        start_bg rviz rviz2 -d "$RVIZ_CONFIG"
    fi
}


# Reabrir RViz conservando la posición, los servos y las turbinas del gemelo.
reopen_rviz() {

    pgrep -f '^[^ ]*python3 .*[g]ecko_twin_node\.py' >/dev/null 2>&1 ||
        fail "El sistema no está en ejecución. Inícialo con: $0 start"

    [[ -f "$WS_DIR/install/setup.bash" ]] ||
        fail "El panel no está compilado. Inicia el sistema con: $0 start"

    set +u
    source "$ROS_SETUP"
    source "$WS_DIR/install/setup.bash"
    set -u

    if pgrep -f '^[^ ]*[r]viz2 -d .*gecko\.rviz' >/dev/null 2>&1; then
        info "RViz ya está abierto."
        return 0
    fi

    rm -f "$PID_DIR/rviz.pid"
    start_rviz
}


# ------------------------------ DETENER ------------------------------

stop_all() {

    info "Deteniendo el sistema Gecko..."

    stop_nodes

    # Eliminar entidades antes de cerrar Gazebo.
    if command -v ign >/dev/null 2>&1 &&
       pgrep -f '[i]gn gazebo' >/dev/null 2>&1 &&
       wait_for_ign_service "/world/$WORLD_NAME/remove" 3; then

        remove_entity_if_present "$WALL_NAME"
        remove_entity_if_present "$ROBOT_NAME"
    fi

    # Cerrar servidor e interfaz gráfica.
    stop_gazebo_processes

    # Detener el emisor de la Raspberry mientras el hotspot sigue activo.
    if [[ "$GECKO_SKIP_PI" != "1" ]]; then
        bash "$SCRIPTS_DIR/gecko_pi.sh" stop || true
    fi

    # Apagar hotspot.
    if [[ "$GECKO_SKIP_HOTSPOT" != "1" ]] && command -v nmcli >/dev/null 2>&1; then
        nmcli connection down "$HOTSPOT_SSID" >/dev/null 2>&1 || true
    fi

    info "Sistema detenido."
}


# ------------------------------ HOTSPOT ------------------------------

start_hotspot() {

    if [[ "$GECKO_SKIP_HOTSPOT" == "1" ]]; then
        info "Hotspot omitido (GECKO_SKIP_HOTSPOT=1)."
        return 0
    fi

    info "Configurando hotspot Wi-Fi $HOTSPOT_SSID..."

    nmcli radio wifi on

    if ! nmcli -t -f NAME connection show |
         grep -Fx "$HOTSPOT_SSID" >/dev/null; then

        nmcli connection add type wifi \
            ifname "$WIFI_IFACE" \
            con-name "$HOTSPOT_SSID" \
            ssid "$HOTSPOT_SSID" >/dev/null
    fi

    nmcli connection modify "$HOTSPOT_SSID" \
        connection.autoconnect no \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        802-11-wireless.channel "$HOTSPOT_CHANNEL" \
        802-11-wireless.powersave 2 \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$HOTSPOT_PASSWORD" \
        ipv4.method shared \
        ipv4.addresses "$HOTSPOT_IP" \
        ipv6.method ignore

    nmcli connection up "$HOTSPOT_SSID" >/dev/null

    local ip_ok=0

    for _ in {1..15}; do

        if ip -4 addr show dev "$WIFI_IFACE" 2>/dev/null |
           grep -Fq "${HOTSPOT_IP%/*}"; then

            ip_ok=1
            break
        fi

        sleep 1
    done

    [[ "$ip_ok" -eq 1 ]] ||
        fail "El hotspot no obtuvo la IP ${HOTSPOT_IP%/*}."

    info "Hotspot activo. IP: ${HOTSPOT_IP%/*}"
}


# ------------------------------- INICIO ------------------------------

main() {

    mkdir -p "$LOG_DIR" "$PID_DIR"
    clean_conda_env

    case "${1:-start}" in
        stop)
            stop_all
            exit 0
            ;;
        rviz)
            reopen_rviz
            exit 0
            ;;
        pi)
            bash "$SCRIPTS_DIR/gecko_pi.sh" start
            exit $?
            ;;
        start)
            ;;
        *)
            fail "Uso: $0 [start|stop|rviz|pi]"
            ;;
    esac


    # Comprobar comandos y archivos.
    for cmd in ign ping ip tail pgrep pkill colcon; do
        require_command "$cmd"
    done

    if [[ "$GECKO_SKIP_HOTSPOT" != "1" ]]; then
        require_command nmcli
    fi

    if [[ "$GECKO_HEADLESS" == "1" ]]; then
        require_command xvfb-run
    fi

    [[ -x "$SYSTEM_PYTHON" ]] ||
        fail "No existe $SYSTEM_PYTHON"

    [[ -f "$ROS_SETUP" ]] ||
        fail "No existe $ROS_SETUP"

    [[ -f "$SOURCE_URDF" ]] ||
        fail "No existe el URDF del vehículo: $SOURCE_URDF"

    [[ -f "$WALL_SDF" ]] ||
        fail "No existe el modelo de pared: $WALL_SDF"

    [[ -f "$RVIZ_CONFIG" ]] ||
        fail "No existe la configuración de RViz: $RVIZ_CONFIG"

    local script
    for script in udp_imu_receiver.py gecko_twin_node.py \
                  imu_displacement_estimator.py wall_climb_manager.py \
                  generate_gecko_models.py; do
        [[ -f "$SCRIPTS_DIR/$script" ]] || fail "Falta $script"
    done


    # Cargar ROS 2 Humble.
    # Desactivar nounset temporalmente para evitar errores de setup.bash.

    set +u
    source "$ROS_SETUP"
    set -u

    require_command ros2


    # Eliminar nodos anteriores para evitar procesos duplicados.

    stop_nodes

    sleep 1


    # ================================================================
    # 1. ACTIVAR HOTSPOT
    # ================================================================

    start_hotspot

    # La Raspberry se busca en segundo plano mientras arranca Gazebo.
    if [[ "$GECKO_SKIP_PI" != "1" ]]; then
        : > "$LOG_DIR/pi_link.log"
        start_bg pi_link bash "$SCRIPTS_DIR/gecko_pi.sh" start
    fi


    # ================================================================
    # 2. PREPARAR MODELOS Y PANEL
    # ================================================================

    info "Generando los modelos del Gecko..."

    "$SYSTEM_PYTHON" "$SCRIPTS_DIR/generate_gecko_models.py" \
        --source "$SOURCE_URDF" --out "$MODELS_DIR" >/dev/null

    build_panel

    set +u
    source "$WS_DIR/install/setup.bash"
    set -u


    # ================================================================
    # 3. CREAR SIEMPRE UN GAZEBO NUEVO
    # ================================================================

    info "Cerrando cualquier instancia anterior de Gazebo..."

    stop_gazebo_processes

    sleep 2

    # Reiniciar registro de Gazebo.
    : > "$LOG_DIR/gazebo.log"

    info "Iniciando un mundo nuevo con empty.sdf..."

    if [[ "$GECKO_HEADLESS" == "1" ]]; then
        start_bg gazebo ign gazebo -s -r empty.sdf
    else
        # -r inicia la simulación sin pausa y abre la interfaz gráfica.
        start_bg gazebo ign gazebo -r empty.sdf
    fi

    info "Esperando los servicios del nuevo mundo..."

    wait_for_ign_service "/world/$WORLD_NAME/create" 60 ||
        fail "Gazebo no publicó el servicio create."

    wait_for_ign_service "/world/$WORLD_NAME/remove" 20 ||
        fail "Gazebo no publicó el servicio remove."

    info "Mundo empty creado correctamente."


    # ================================================================
    # 4. INSERTAR EL GECKO
    # ================================================================

    info "Insertando el modelo $ROBOT_NAME..."

    local create_result

    create_result=$(
    ign service \
        -s "/world/$WORLD_NAME/create" \
        --reqtype ignition.msgs.EntityFactory \
        --reptype ignition.msgs.Boolean \
        --timeout 15000 \
        --req "sdf_filename: \"$ROBOT_URDF\", name: \"$ROBOT_NAME\""
    ) || fail "No se pudo insertar el vehículo."

    printf '%s\n' "$create_result"

    grep -q 'data: true' <<<"$create_result" ||
    fail "Gazebo no confirmó la creación del vehículo."

    sleep 2

    if [[ "$GECKO_HEADLESS" != "1" ]]; then
        focus_gazebo_camera
    fi


    # ================================================================
    # 5. PUENTE ROS 2 - GAZEBO
    # ================================================================

    info "Iniciando puente ROS 2 - Gazebo..."

    local -a joint_bridges

    mapfile -t joint_bridges < <(
        "$SYSTEM_PYTHON" "$SCRIPTS_DIR/generate_gecko_models.py" --print-bridge-args
    )

    start_bg bridge "$BRIDGE_BIN" \
        "/world/$WORLD_NAME/set_pose@ros_gz_interfaces/srv/SetEntityPose" \
        "/world/$WORLD_NAME/create@ros_gz_interfaces/srv/SpawnEntity" \
        "/world/$WORLD_NAME/remove@ros_gz_interfaces/srv/DeleteEntity" \
        "${joint_bridges[@]}"

    local service_ok=0

    for _ in {1..30}; do

        if ros2 service list 2>/dev/null |
           grep -Fx "/world/$WORLD_NAME/set_pose" >/dev/null; then

            service_ok=1
            break
        fi

        sleep 1
    done

    [[ "$service_ok" -eq 1 ]] ||
        fail "El puente no publicó /world/$WORLD_NAME/set_pose."


    # ================================================================
    # 6. NODOS DEL GEMELO
    # ================================================================

    info "Iniciando receptor UDP..."

    start_bg receiver "$SYSTEM_PYTHON" "$SCRIPTS_DIR/udp_imu_receiver.py"

    info "Iniciando el gemelo digital a 30 Hz..."

    start_bg twin "$SYSTEM_PYTHON" "$SCRIPTS_DIR/gecko_twin_node.py"

    # El gestor de pared consume /imu/estimated_pitch_deg.
    # La posición estimada por la IMU NO se aplica al vehículo.

    info "Iniciando estimador de Pitch..."

    start_bg estimator "$SYSTEM_PYTHON" "$SCRIPTS_DIR/imu_displacement_estimator.py"

    info "Iniciando gestor de pared..."

    start_bg wall_manager "$SYSTEM_PYTHON" "$SCRIPTS_DIR/wall_climb_manager.py"


    # ================================================================
    # 7. RVIZ CON EL PANEL DE CONTROL
    # ================================================================

    info "Iniciando RViz..."

    start_bg robot_state_publisher "$RSP_BIN" --ros-args --params-file "$RSP_PARAMS"

    start_rviz


    # ================================================================
    # 8. RESUMEN
    # ================================================================

    info "Sistema Gecko iniciado correctamente."

    printf '\n'
    if [[ "$GECKO_SKIP_HOTSPOT" == "1" ]]; then
        printf '  Hotspot:         omitido\n'
    else
        printf '  Hotspot:         %s\n' "$HOTSPOT_SSID"
        printf '  IP del PC:       %s\n' "${HOTSPOT_IP%/*}"
    fi
    if [[ "$GECKO_SKIP_PI" == "1" ]]; then
        printf '  Raspberry:       omitida\n'
    else
        printf '  Raspberry:       se inicia por SSH en segundo plano\n'
        printf '                   (estado: %s/pi_link.log)\n' "$LOG_DIR"
    fi
    printf '  Mundo:           empty.sdf NUEVO\n'
    printf '  Vehículo:        %s\n' "$ROBOT_NAME"
    printf '  IMU:             UDP 5005 -> /imu/data\n'
    printf '  Orientación:     IMU real, 30 Hz\n'
    printf '  Motores:         panel "Gecko Control" en RViz\n'
    printf '  Pared:           wall_climb_manager.py\n'
    printf '\n'
    printf '  Registros:    %s\n' "$LOG_DIR"
    printf '  Reabrir RViz: %s rviz\n' "$0"
    printf '  Reintentar Raspberry: %s pi\n' "$0"
    printf '  Detener:      %s stop\n' "$0"
}

main "$@"
