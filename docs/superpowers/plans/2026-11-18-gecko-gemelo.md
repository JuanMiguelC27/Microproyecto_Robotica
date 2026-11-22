# Gecko: gemelo digital. Plan de implementación

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convertir `mini_sumo_imu` en el gemelo digital del Gecko: modelo real en Gazebo, orientación desde la IMU, y motores y desplazamiento controlados desde un panel propio en RViz.

**Architecture:** Un nodo Python (`gecko_twin_node.py`) es la única autoridad sobre la pose y los joints. Su lógica de contacto y movimiento vive en módulos puros sin ROS (`gecko_geometry.py`, `gecko_motion.py`). Un panel C++ de RViz publica los comandos, y `ros_gz_bridge` los lleva a los controladores de joints de Gazebo.

**Tech Stack:** ROS 2 Humble, Gazebo Fortress (ign-gazebo 6), RViz 2, C++17 con Qt5 y pluginlib, Python 3.10 (`/usr/bin/python3`), pytest 6.2, gtest.

**Spec:** `docs/superpowers/specs/2026-11-18-gecko-gemelo-design.md`

## Global Constraints

- Todo Python se ejecuta con `/usr/bin/python3`. El `python3` del PATH es Miniconda 3.14 y no carga `rclpy`.
- No modificar `Micro2_URDFV2/` ni nada dentro de `Macroproyecto/`.
- El panel no envía nada al vehículo real ni a la Raspberry.
- No hacer commits ni push: el usuario no los ha pedido. Los pasos de este plan terminan en verificación.
- Nombres oficiales: entidad `gecko`, pared `gecko_wall`, hotspot `Gecko-IMU`, clave `Gecko2026!`, mundo `empty`.
- Velocidad lineal `0.15` m/s, radio de rueda `0.015` m, rueda `10` rad/s, turbina máxima `60` rad/s, servos `±0.7854` rad, descenso `0.5` m/s.
- Pared: `0.10 × 2.0 × 2.0` m, centro a `z = 1.0`, entra con pitch ≤ `−60°` durante `0.30 s`, sale con pitch ≥ `−50°` durante `0.50 s`.
- Ciclo del gemelo: `30` Hz, `dt` limitado a `0.1 s`. Watchdog del joystick: `0.5 s`.
- Las pruebas de integración usan `ROS_DOMAIN_ID=77` e `IGN_PARTITION=gecko_test`.

## Review Focus

1. **Comandos inválidos** (NaN, infinito, fuera de rango, `drive` desconocido): se ignoran o se limitan, y el gemelo nunca recibe una pose inválida. Prueba en Task 5.
2. **RViz se cierra con el joystick sostenido:** las ruedas paran en `0.5 s`, y servos y turbinas siguen igual. Pruebas en Task 5 y Task 6.
3. **RViz se reabre con servos o turbinas activos:** los controles muestran los valores vigentes y no los reinician a 0. Pruebas en Task 6 y Task 8.
4. **Gazebo rechaza crear la pared:** el gemelo sigue en PISO y el gestor reintenta al cumplirse de nuevo la condición. Prueba en Task 7.
5. **`start` repetido, `stop` repetido y Conda activo:** sin procesos duplicados, sin errores y con los nodos en Python 3.10. Prueba en Task 9.

## Estructura de archivos

Rutas relativas a `IMU/gecko_imu/`.

| Archivo | Responsabilidad |
|---|---|
| `start_gecko_pc.sh` | Arranque y parada del sistema |
| `scripts/gecko_joints.py` | Nombres de joints, tópicos de Gazebo y argumentos del puente |
| `scripts/generate_gecko_models.py` | Genera los URDF de RViz y Gazebo y los parámetros de `robot_state_publisher` |
| `scripts/gecko_geometry.py` | Cuaterniones, envolvente y función de apoyo `support` |
| `scripts/gecko_motion.py` | Estados PISO, PARED y DESCENSO, y saneo de comandos |
| `scripts/gecko_twin_node.py` | Nodo del gemelo (evoluciona desde `imu_to_gazebo.py`) |
| `scripts/wall_climb_manager.py` | Crea y elimina la pared en la pose indicada por el gemelo |
| `ros2_ws/src/gecko_rviz_panel/` | Panel C++ de RViz |
| `rviz/gecko.rviz` | Configuración de RViz |
| `tests/unit/` | Pruebas sin ROS |
| `tests/integration/` | Pruebas con ROS y Gazebo |
| `tests/run_ros.sh` | Ejecuta un comando en un entorno ROS limpio de Conda |

---

### Task 1: Respaldo y renombrado

**Files:**
- Create: `IMU/respaldo_pre_gecko_2026-11-18.tar.gz`, `IMU/gecko_imu/tests/conftest.py`, `IMU/gecko_imu/tests/unit/test_naming.py`
- Modify: todo `IMU/mini_sumo_imu/`, que pasa a `IMU/gecko_imu/`
- Rename: `start_mini_sumo_pc.sh` → `start_gecko_pc.sh`

**Interfaces:**
- Produces: la carpeta `IMU/gecko_imu/`. `tests/conftest.py` añade `scripts/` a `sys.path`.

- [ ] **Step 1: Crear el respaldo y comprobarlo**

Run: `cd IMU && tar -czf respaldo_pre_gecko_2026-11-18.tar.gz mini_sumo_imu && tar -tzf respaldo_pre_gecko_2026-11-18.tar.gz | grep -c start_mini_sumo_pc.sh`
Expected: `1`

- [ ] **Step 2: Mover la carpeta, renombrar el script y borrar `__pycache__`**

Run: `mv IMU/mini_sumo_imu IMU/gecko_imu && mv IMU/gecko_imu/start_mini_sumo_pc.sh IMU/gecko_imu/start_gecko_pc.sh && find IMU/gecko_imu -name __pycache__ -type d -prune -exec rm -rf {} +`
Expected: sin salida. `IMU/mini_sumo_imu.zip` no se toca.

- [ ] **Step 3: Crear `tests/conftest.py`, que añade `scripts/` a `sys.path`, y escribir la prueba de nombres**

```python
# tests/unit/test_naming.py
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
NEEDLE = 'su' + 'mo'
SKIP = {'build', 'install', 'log', '__pycache__', '.pytest_cache'}

def project_files():
    for path in ROOT.rglob('*'):
        if path.is_file() and not SKIP & set(path.relative_to(ROOT).parts):
            yield path

def test_no_file_uses_old_name():
    assert [p for p in project_files() if NEEDLE in p.name.lower()] == []

def test_no_content_uses_old_name():
    hits = [p for p in project_files()
            if NEEDLE in p.read_text(errors='ignore').lower()]
    assert hits == []
```

- [ ] **Step 4: Ejecutar y comprobar que falla**

Run: `cd IMU/gecko_imu && /usr/bin/python3 -m pytest tests/unit/test_naming.py -v`
Expected: FAIL en `test_no_content_uses_old_name`, con los scripts listados.

- [ ] **Step 5: Reemplazar los textos**

En todos los archivos de `IMU/gecko_imu/`: `MiniSumo-IMU` → `Gecko-IMU`, `MiniSumo2026!` → `Gecko2026!`, `mini_sumo_imu` → `gecko_imu`, `mini_sumo_wall` → `gecko_wall`, `start_mini_sumo_pc` → `start_gecko_pc`, `mini_sumo` → `gecko`, `Mini Sumo` → `Gecko`. La variable `ROBOT_URDF` pasa a `$PROJECT_DIR/models/gecko/gecko_gazebo.urdf`.

- [ ] **Step 6: Ejecutar y comprobar que pasa**

Run: `cd IMU/gecko_imu && /usr/bin/python3 -m pytest tests/unit/test_naming.py -v && bash -n start_gecko_pc.sh`
Expected: 2 passed, sin errores de sintaxis.

---

### Task 2: Generador de modelos

**Files:**
- Create: `scripts/gecko_joints.py`, `scripts/generate_gecko_models.py`, `tests/unit/test_models.py`

**Interfaces:**
- Produces en `gecko_joints.py`:
  - `WHEEL_JOINTS = ('Wheel_FL_Joint', 'Wheel_FR_Joint', 'Wheel_RL_Joint', 'Wheel_RR_Joint')`
  - `TURBINE_JOINTS = ('Turbine_F_Joint', 'Turbine_R_Joint')`
  - `SERVO_JOINTS = ('Turbine_Servo_F_Joint', 'Turbine_Servo_R_Joint')`
  - `velocity_topic(joint: str) -> str`, que devuelve `/gecko/gz/<joint>/cmd_vel`
  - `position_topic(joint: str) -> str`, que devuelve `/gecko/gz/<joint>/cmd_pos`
  - `bridge_args() -> list[str]`, 8 cadenas `<topic>@std_msgs/msg/Float64]ignition.msgs.Double`
- Produces en `generate_gecko_models.py`:
  - `generate(source_urdf: Path, out_dir: Path) -> dict[str, Path]`, con claves `rviz`, `gazebo`, `rsp_params`
  - CLI: `--source <urdf> --out <dir>` y `--print-bridge-args`, que imprime un argumento por línea

- [ ] **Step 1: Escribir las pruebas**

```python
# tests/unit/test_models.py
import hashlib, subprocess, xml.etree.ElementTree as ET
from pathlib import Path
import yaml
import gecko_joints as gj
from generate_gecko_models import generate

SOURCE = Path(__file__).resolve().parents[4] / 'Micro2_URDFV2/urdf/Micro2_URDFV2.urdf'

def models(tmp_path): return generate(SOURCE, tmp_path)

def test_source_is_not_modified(tmp_path):
    before = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    models(tmp_path)
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == before

def test_rviz_meshes_are_existing_file_uris(tmp_path):
    root = ET.parse(models(tmp_path)['rviz']).getroot()
    names = [m.get('filename') for m in root.iter('mesh')]
    visuals = root.findall('./link/visual')
    assert len(visuals) == 9
    assert all(n.startswith('file://') and Path(n[7:]).is_file() for n in names)

def test_gazebo_has_no_collisions_and_no_gravity(tmp_path):
    root = ET.parse(models(tmp_path)['gazebo']).getroot()
    assert list(root.iter('collision')) == []
    refs = {g.get('reference') for g in root.findall('gazebo') if g.find('gravity') is not None}
    assert refs == {l.get('name') for l in root.findall('link')}

def test_gazebo_plugins(tmp_path):
    root = ET.parse(models(tmp_path)['gazebo']).getroot()
    names = [p.get('name') for p in root.iter('plugin')]
    assert names.count('ignition::gazebo::systems::JointController') == 6
    assert names.count('ignition::gazebo::systems::JointPositionController') == 2
    assert names.count('ignition::gazebo::systems::JointStatePublisher') == 1

def test_gazebo_model_converts_to_sdf(tmp_path):
    out = subprocess.run(['ign', 'sdf', '-p', str(models(tmp_path)['gazebo'])],
                         capture_output=True, text=True, check=True).stdout
    assert out.count('<joint name=') == 8
    assert out.count('<collision') == 0

def test_rsp_params_contain_description(tmp_path):
    data = yaml.safe_load(models(tmp_path)['rsp_params'].read_text())
    assert '<robot' in data['robot_state_publisher']['ros__parameters']['robot_description']

def test_bridge_args():
    args = gj.bridge_args()
    assert len(args) == 8 and all(a.endswith('@std_msgs/msg/Float64]ignition.msgs.Double') for a in args)
```

- [ ] **Step 2: Ejecutar y comprobar que falla**

Run: `cd IMU/gecko_imu && /usr/bin/python3 -m pytest tests/unit/test_models.py -v`
Expected: FAIL con `ModuleNotFoundError: gecko_joints`.

- [ ] **Step 3: Implementar `gecko_joints.py` y `generate_gecko_models.py`**

Usar `xml.etree.ElementTree`. Las mallas se resuelven desde `source_urdf.parent.parent / 'meshes'`. En RViz se escriben como `file://<ruta absoluta>` y en Gazebo como ruta absoluta sin esquema.

Plugins de Gazebo dentro de `<gazebo>`:

| Plugin | Archivo | Parámetros |
|---|---|---|
| `JointController` | `ignition-gazebo-joint-controller-system` | `joint_name`, `topic = velocity_topic(joint)` |
| `JointPositionController` | `ignition-gazebo-joint-position-controller-system` | `joint_name`, `topic = position_topic(joint)`, `use_velocity_commands = true`, `p_gain = 10`, `i_gain = 0`, `d_gain = 0`, `cmd_max = 5`, `cmd_min = -5` |
| `JointStatePublisher` | `ignition-gazebo-joint-state-publisher-system` | ninguno |

Cada link recibe `<gazebo reference="<link>"><gravity>false</gravity></gazebo>`.

- [ ] **Step 4: Ejecutar y comprobar que pasa**

Run: `cd IMU/gecko_imu && /usr/bin/python3 -m pytest tests/unit/test_models.py -v`
Expected: 7 passed.

---

### Task 3: Prototipo de `set_pose` con joints (punto de decisión)

**Files:**
- Create: `tests/run_ros.sh`, `tests/integration/check_joints_under_set_pose.py`

**Interfaces:**
- Consumes: `gecko_joints`, `generate()` (Task 2).
- Produces:
  - `tests/run_ros.sh <comando...>`: quita Miniconda del `PATH`, borra `CONDA_*`, `PYTHONHOME` y `PYTHONPATH`, fija `ROS_DOMAIN_ID=77` e `IGN_PARTITION=gecko_test`, carga ROS Humble y, si existe, `ros2_ws/install/setup.bash`, y ejecuta el comando.
  - `check_joints_under_set_pose.py`: termina con código 0 si cumple los criterios y 1 si no.

- [ ] **Step 1: Implementar el verificador**

El script genera los modelos en un directorio temporal, lanza `ign gazebo -s -r empty.sdf`, crea la entidad `gecko` y arranca el puente con `bridge_args()`, `/world/empty/set_pose` y `/world/empty/model/gecko/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model`.

Durante 10 s llama a `set_pose` a 30 Hz con pitch `30°·sin(2π·0.5·t)`, y comanda ruedas a `10` rad/s, turbinas a `60` rad/s y servos a `0.5` rad. Mide los últimos 8 s y cierra todos los procesos al terminar.

- [ ] **Step 2: Ejecutar el verificador**

Run: `cd IMU/gecko_imu && tests/run_ros.sh /usr/bin/python3 tests/integration/check_joints_under_set_pose.py`
Expected: código 0 e impresión de `wheels_ok turbines_ok servos_ok`, con:
- Velocidad media de cada rueda: `10 ± 1` rad/s.
- Velocidad media de cada turbina: `60 ± 6` rad/s.
- Posición final de cada servo: `0.5 ± 0.0175` rad.

- [ ] **Step 3: Aplicar la decisión**

Si pasa, continuar con Task 4. Si falla tras ajustar las ganancias de los servos, **detenerse y reportar las medidas al usuario** antes de seguir, porque la alternativa cambia la arquitectura.

---

### Task 4: Geometría y función de apoyo

**Files:**
- Create: `scripts/gecko_geometry.py`, `tests/unit/test_geometry.py`

**Interfaces:**
- Produces:
  - `Vec3 = tuple[float, float, float]`, `Quat = tuple[float, float, float, float]` (orden `x, y, z, w`), `IDENTITY: Quat`
  - `q_from_rpy(roll: float, pitch: float, yaw: float) -> Quat`
  - `rotate(q: Quat, v: Vec3) -> Vec3`
  - `support(q: Quat, u: Vec3) -> float`

Envolvente exacta:

| Elemento | Valores |
|---|---|
| Ruedas (radio `0.015`, eje Y) | `(0.142, 0.12909, -0.013419)`, `(0.142, -0.12909, -0.011481)`, `(-0.14196, -0.12909, -0.012381)`, `(-0.142, 0.12909, -0.013381)` |
| Chasis (8 esquinas) | `x = ±0.153`, `y = ±0.1308`, `z ∈ {-0.0209, 0.068}` |
| Servos y turbinas (radio `0.056`, eje Y) | `(±0.09, ±0.0672, 0.0415)`, 4 discos |

- [ ] **Step 1: Escribir las pruebas**

```python
# tests/unit/test_geometry.py
from math import radians
from pytest import approx
from gecko_geometry import IDENTITY, q_from_rpy, rotate, support

UP, DOWN, FRONT, BACK, LEFT = (0,0,1), (0,0,-1), (1,0,0), (-1,0,0), (0,1,0)

def test_nose_up_is_negative_pitch():
    assert rotate(q_from_rpy(0, radians(-90), 0), FRONT) == approx((0, 0, 1), abs=1e-9)

def test_level_support():
    assert support(IDENTITY, DOWN) == approx(0.028419)
    assert support(IDENTITY, UP) == approx(0.0975)
    assert support(IDENTITY, FRONT) == approx(0.157)
    assert support(IDENTITY, BACK) == approx(0.157)
    assert support(IDENTITY, LEFT) == approx(0.1308)

def test_vertical_support():
    q = q_from_rpy(0, radians(-90), 0)
    assert support(q, DOWN) == approx(0.157)
    assert support(q, FRONT) == approx(0.028419)
```

- [ ] **Step 2: Ejecutar y comprobar que falla**

Run: `cd IMU/gecko_imu && /usr/bin/python3 -m pytest tests/unit/test_geometry.py -v`
Expected: FAIL con `ModuleNotFoundError: gecko_geometry`.

- [ ] **Step 3: Implementar `gecko_geometry.py`**

Aporte de un disco de centro `c`, radio `r` y eje de cuerpo `a`: `dot(rotate(q, c), u) + r * sqrt(max(0, 1 - dot(rotate(q, a), u)**2))`. Aporte de una esquina `b`: `dot(rotate(q, b), u)`. `support` devuelve el máximo.

- [ ] **Step 4: Ejecutar y comprobar que pasa**

Run: `cd IMU/gecko_imu && /usr/bin/python3 -m pytest tests/unit/test_geometry.py -v`
Expected: 3 passed.

---

### Task 5: Movimiento y saneo de comandos

**Files:**
- Create: `scripts/gecko_motion.py`, `tests/unit/test_motion.py`

**Interfaces:**
- Consumes: `gecko_geometry` (Task 4).
- Produces:
  - Constantes `DRIVE_STOP = 0`, `DRIVE_FORWARD = 1`, `DRIVE_BACK = 2`, `DRIVE_RIGHT = 3`, `DRIVE_LEFT = 4`
  - `class Mode(Enum)`: `FLOOR`, `WALL`, `DESCENT`
  - `@dataclass(frozen=True) class Wall`: `heading: Vec3`, `face: Vec3`
  - `wall_from_center(center: Vec3, yaw: float) -> Wall`, con `face = center − heading·0.05`
  - `class GeckoMotion(speed=0.15, descent_speed=0.5, wall_top=2.0, wall_half_width=1.0)` con atributos `position: Vec3`, `mode: Mode`, `heading: Vec3` y métodos:
    - `step(q: Quat, drive: int, dt: float) -> Vec3`
    - `set_wall(wall: Optional[Wall]) -> None`
    - `wall_target(q: Quat) -> tuple[Vec3, float]`, que devuelve centro de pared y guiñada
  - `sanitize_drive(value: int) -> int`
  - `clamp_servo(rad: float, current: float) -> float`
  - `clamp_turbine(value: float, current: float) -> float`
  - `effective_drive(drive: int, age_s: float, timeout: float = 0.5) -> int`
  - `wheel_velocities(drive: int, omega: float = 10.0) -> dict[str, float]`

- [ ] **Step 1: Escribir las pruebas**

```python
# tests/unit/test_motion.py
from math import radians, nan, inf, cos, sin
import pytest
from pytest import approx
from gecko_geometry import IDENTITY, q_from_rpy, support
from gecko_motion import *

DOWN, UP = (0,0,-1), (0,0,1)
NOSE_UP_75 = q_from_rpy(0, radians(-75), 0)
VERTICAL = q_from_rpy(0, radians(-90), 0)

def run(m, q, drive, seconds, dt=0.1):
    for _ in range(round(seconds / dt)):
        m.step(q, drive, dt)

def on_wall(q=VERTICAL):
    m = GeckoMotion()
    m.step(q, DRIVE_STOP, 0.033)
    m.set_wall(Wall(heading=(1,0,0), face=(1,0,0)))
    m.step(q, DRIVE_STOP, 0.033)
    return m

def test_starts_on_floor():
    assert GeckoMotion().position == approx((0, 0, 0.028419))

def test_forward_and_back_at_constant_speed():
    m = GeckoMotion(); run(m, IDENTITY, DRIVE_FORWARD, 1.0)
    assert m.position[0] == approx(0.15)
    run(m, IDENTITY, DRIVE_BACK, 1.0)
    assert m.position[0] == approx(0.0, abs=1e-9)

def test_turning_only_spins_wheels():
    m = GeckoMotion(); run(m, IDENTITY, DRIVE_RIGHT, 1.0)
    assert m.position[:2] == approx((0, 0))

def test_moves_along_imu_heading():
    m = GeckoMotion(); run(m, q_from_rpy(0, 0, radians(90)), DRIVE_FORWARD, 1.0)
    assert m.position[:2] == approx((0, 0.15), abs=1e-9)

@pytest.mark.parametrize('pitch', [-60, -30, 30, 60])
def test_tilted_motion_stays_horizontal(pitch):
    q = q_from_rpy(0, radians(pitch), 0)
    m = GeckoMotion(); run(m, q, DRIVE_FORWARD, 1.0)
    assert m.position[0] == approx(0.15)
    assert m.position[2] == approx(support(q, DOWN))

def test_near_vertical_keeps_last_heading():
    m = GeckoMotion(); m.step(q_from_rpy(0, 0, radians(90)), DRIVE_STOP, 0.1)
    run(m, q_from_rpy(0, radians(-89.9), radians(90)), DRIVE_FORWARD, 1.0)
    assert m.position[:2] == approx((0, 0.15), abs=1e-6)

def test_never_flies_or_sinks_in_any_orientation():
    m = GeckoMotion()
    for r in range(-90, 91, 15):
        for p in range(-90, 91, 15):
            q = q_from_rpy(radians(r), radians(p), radians(25))
            m.step(q, DRIVE_FORWARD, 0.033)
            assert abs(m.position[2] - support(q, DOWN)) < 1e-3

def test_large_dt_is_limited():
    m = GeckoMotion(); m.step(IDENTITY, DRIVE_FORWARD, 5.0)
    assert m.position[0] == approx(0.015)

def test_non_positive_dt_does_not_move():
    m = GeckoMotion(); m.step(IDENTITY, DRIVE_FORWARD, -1.0)
    assert m.position[0] == 0.0

def test_wall_target_is_in_front():
    center, yaw = GeckoMotion().wall_target(IDENTITY)
    assert center == approx((0.217, 0, 1.0)) and yaw == approx(0.0)

def test_wall_from_center():
    w = wall_from_center((2.0, 1.0, 1.0), radians(90))
    assert w.heading == approx((0, 1, 0), abs=1e-9)
    assert w.face == approx((2.0, 0.95, 1.0))

def test_wall_contact_distance():
    m = on_wall()
    assert m.mode is Mode.WALL and m.position[0] == approx(1 - 0.028419)

def test_climbs_and_descends_wall():
    m = on_wall(); z0 = m.position[2]
    run(m, VERTICAL, DRIVE_FORWARD, 1.0); assert m.position[2] == approx(z0 + 0.15)
    run(m, VERTICAL, DRIVE_BACK, 1.0); assert m.position[2] == approx(z0)

def test_wall_limits():
    m = on_wall()
    run(m, VERTICAL, DRIVE_FORWARD, 20.0); assert m.position[2] == approx(2.0 - 0.157)
    run(m, VERTICAL, DRIVE_BACK, 20.0); assert m.position[2] == approx(0.157)

def test_lateral_limit_on_wall():
    q = q_from_rpy(0, radians(-75), radians(60))
    m = on_wall(q); run(m, q, DRIVE_FORWARD, 30.0)
    assert abs(m.position[1]) <= 1.0 - support(q, (0, 1, 0)) + 1e-9

def test_descent_ignores_joystick_then_lands():
    m = on_wall(); run(m, VERTICAL, DRIVE_FORWARD, 5.0)
    x, z = m.position[0], m.position[2]
    m.set_wall(None); m.step(VERTICAL, DRIVE_FORWARD, 0.1)
    assert m.mode is Mode.DESCENT
    assert m.position[0] == approx(x) and m.position[2] == approx(z - 0.05)
    run(m, IDENTITY, DRIVE_STOP, 10.0)
    assert m.mode is Mode.FLOOR and m.position[2] == approx(0.028419)

def test_wheel_signs():
    fwd = wheel_velocities(DRIVE_FORWARD); assert set(fwd.values()) == {10.0}
    right = wheel_velocities(DRIVE_RIGHT)
    assert right['Wheel_FL_Joint'] == right['Wheel_RL_Joint'] == 10.0
    assert right['Wheel_FR_Joint'] == right['Wheel_RR_Joint'] == -10.0
    left = wheel_velocities(DRIVE_LEFT)
    assert left['Wheel_FL_Joint'] == -10.0 and left['Wheel_FR_Joint'] == 10.0
    assert set(wheel_velocities(DRIVE_BACK).values()) == {-10.0}
    assert set(wheel_velocities(DRIVE_STOP).values()) == {0.0}

# Review Focus 1
@pytest.mark.parametrize('bad', [-1, 5, 99])
def test_unknown_drive_means_stop(bad):
    assert sanitize_drive(bad) == DRIVE_STOP

@pytest.mark.parametrize('bad', [nan, inf, -inf])
def test_invalid_values_keep_current(bad):
    assert clamp_servo(bad, 0.3) == 0.3 and clamp_turbine(bad, -0.4) == -0.4

def test_values_are_limited():
    assert clamp_servo(2.0, 0.0) == 0.7854 and clamp_servo(-2.0, 0.0) == -0.7854
    assert clamp_turbine(3.0, 0.0) == 1.0 and clamp_turbine(-3.0, 0.0) == -1.0

# Review Focus 2
def test_watchdog_stops_drive():
    assert effective_drive(DRIVE_FORWARD, 0.49) == DRIVE_FORWARD
    assert effective_drive(DRIVE_FORWARD, 0.51) == DRIVE_STOP
```

- [ ] **Step 2: Ejecutar y comprobar que falla**

Run: `cd IMU/gecko_imu && /usr/bin/python3 -m pytest tests/unit/test_motion.py -v`
Expected: FAIL con `ModuleNotFoundError: gecko_motion`.

- [ ] **Step 3: Implementar `gecko_motion.py`**

Seguir la sección 9 del spec. Reglas que el spec no fija:

- `dt <= 0` no mueve, y `dt` se limita a `0.1`.
- `heading` se actualiza en cada `step`, en cualquier estado, si la proyección horizontal de `R x̂` mide al menos `0.1`.
- `set_wall(wall)` pasa a `WALL`. `set_wall(None)` pasa a `DESCENT` solo si el estado era `WALL`.
- En `WALL`, si la proyección de `R x̂` sobre el plano de la pared mide menos de `0.1`, no hay avance.
- Coordenada lateral: `a = (p − face)·l`, limitada a `[−half + S(−l), half − S(l)]`.
- `DESCENT` pasa a `FLOOR` cuando `z − descent_speed·dt <= S(−Ẑ)`, y fija `z = S(−Ẑ)`.

- [ ] **Step 4: Ejecutar y comprobar que pasa**

Run: `cd IMU/gecko_imu && /usr/bin/python3 -m pytest tests/unit/test_motion.py -v`
Expected: todas las pruebas passed.

---

### Task 6: Nodo del gemelo

**Files:**
- Rename y modify: `scripts/imu_to_gazebo.py` → `scripts/gecko_twin_node.py`
- Create: `tests/integration/test_twin_node.py`

**Interfaces:**
- Consumes: `gecko_motion`, `gecko_geometry`, `gecko_joints`.
- Produces: nodo `gecko_twin_node` con estos tópicos, además de los de la sección 7 del spec:
  - `/gecko/state/controls`: `std_msgs/Float64MultiArray` `[servo_rad, turbine_f, turbine_r]`, reliable y transient local, publicado al cambiar un valor y al iniciar.
  - `/gecko/wall/target_pose`: publicado en cada ciclo, con `frame_id = world`.

- [ ] **Step 1: Escribir las pruebas de integración**

`tests/integration/test_twin_node.py` lanza el nodo como subproceso con `/usr/bin/python3` y usa un nodo `rclpy` de prueba. Sin Gazebo: el nodo solo avisa que falta `set_pose`.

| Prueba | Acción | Comprobación |
|---|---|---|
| `test_works_without_imu` | Ninguna durante 1 s | TF `world → Base_link` con `z ≈ 0.028419` y rotación identidad |
| `test_forward_moves_base_and_wheels` | `drive = 1` a 10 Hz durante 2 s | `x` entre `0.2` y `0.4`; las 4 ruedas con velocidad `10.0` en `/joint_states` |
| `test_servo_moves_both_joints` | `servo_angle = 0.5` | Ambos servos en `0.5` en `/joint_states` |
| `test_turbines_are_independent_and_hold` | `turbine_f = 0.5`, `turbine_r = -1.0`, y 3 s sin publicar | Velocidades `30.0` y `-60.0` mantenidas |
| `test_watchdog_keeps_servo_and_turbines` | `drive = 1`, `servo = 0.5`, `turbine_f = 0.5`, y dejar de publicar `drive` 1 s | Ruedas en `0.0`; servo y turbina sin cambio |
| `test_late_subscriber_gets_controls` | Fijar servo y turbinas, luego crear un suscriptor nuevo transient local | Recibe `[0.5, 0.5, -1.0]` (Review Focus 3) |
| `test_wall_topics_enter_wall_mode` | Publicar IMU nivelada (la primera muestra calibra el cero), luego pitch −90°, `/gecko/wall/pose` y `/gecko/wall/active = true` | `x = face − 0.028419` |

- [ ] **Step 2: Ejecutar y comprobar que falla**

Run: `cd IMU/gecko_imu && tests/run_ros.sh /usr/bin/python3 -m pytest tests/integration/test_twin_node.py -v`
Expected: FAIL porque `scripts/gecko_twin_node.py` no existe.

- [ ] **Step 3: Implementar el nodo**

Conservar de `imu_to_gazebo.py` la calibración, `/imu/calibrate`, los parámetros `imu_mount_*` e `imu_offset_*`, el TF estático y el patrón `request_in_flight`. Eliminar los parámetros `x`, `y`, `z`.

En cada ciclo de 30 Hz:

1. Calcular la orientación relativa, o `IDENTITY` si no hay IMU.
2. Obtener `effective_drive` y llamar a `motion.step`.
3. Integrar la posición de ruedas y turbinas.
4. Publicar TF, `/joint_states`, `target_pose` y los 8 comandos de Gazebo.
5. Llamar a `set_pose`.

Parámetros: `speed = 0.15`, `wheel_radius = 0.015`, `turbine_max_rad_s = 60.0`, `update_rate_hz = 30.0`. `omega = speed / wheel_radius`. La pared se aplica solo cuando ya existen `active = true` y una pose recibida.

- [ ] **Step 4: Ejecutar y comprobar que pasa**

Run: `cd IMU/gecko_imu && tests/run_ros.sh /usr/bin/python3 -m pytest tests/integration/test_twin_node.py -v`
Expected: 7 passed.

---

### Task 7: Gestor de pared

**Files:**
- Modify: `scripts/wall_climb_manager.py`
- Create: `tests/integration/test_wall_manager.py`

**Interfaces:**
- Consumes: `/gecko/wall/target_pose` (Task 6), `/imu/estimated_pitch_deg`.
- Produces: `/gecko/wall/pose`, `/gecko/wall/active` (ambos reliable y transient local) y `/gecko/wall/marker`.

- [ ] **Step 1: Escribir las pruebas**

`test_wall_manager.py` lanza el nodo como subproceso y ofrece servicios falsos `/world/empty/create` y `/world/empty/remove`.

| Prueba | Acción | Comprobación |
|---|---|---|
| `test_spawns_at_target_pose` | `target_pose` en `(1, 2, 1)` con guiñada 90°, y pitch −75° durante 0,5 s | La petición usa esa pose y el nombre `gecko_wall`; `active = true`; `/gecko/wall/pose` coincide; llega un marker `ADD` tipo `CUBE` de `0.10 × 2.0 × 2.0` |
| `test_waits_for_target_pose` | Pitch −75° sin `target_pose` | No llama a `create`; lo hace al llegar la pose |
| `test_rejected_spawn_stays_inactive` | `create` responde `success = false` | `active` nunca es `true`, y hay un segundo intento con la condición aún cumplida (Review Focus 4) |
| `test_removes_wall_on_exit` | Con pared activa, pitch −40° durante 0,7 s | Llama a `remove` con tipo `2`; `active = false`; marker `DELETE` |

- [ ] **Step 2: Ejecutar y comprobar que falla**

Run: `cd IMU/gecko_imu && tests/run_ros.sh /usr/bin/python3 -m pytest tests/integration/test_wall_manager.py -v`
Expected: FAIL en `test_spawns_at_target_pose`, porque usa la pose fija `0.65, 0, 1`.

- [ ] **Step 3: Implementar los cambios**

Quitar `wall_x`, `wall_y` y `wall_z`. Sin `target_pose`, mantener `condition_since` para reintentar. Publicar `pose` antes que `active`. Tras un rechazo, volver a `FLOOR` con `condition_since = None`.

- [ ] **Step 4: Ejecutar y comprobar que pasa**

Run: `cd IMU/gecko_imu && tests/run_ros.sh /usr/bin/python3 -m pytest tests/integration/test_wall_manager.py -v`
Expected: 4 passed.

---

### Task 8: Panel de RViz

**Files:**
- Create en `ros2_ws/src/gecko_rviz_panel/`: `package.xml`, `CMakeLists.txt`, `plugin_description.xml`, `include/gecko_rviz_panel/drive_mapping.hpp`, `include/gecko_rviz_panel/joystick_widget.hpp`, `include/gecko_rviz_panel/gecko_panel.hpp`, `src/joystick_widget.cpp`, `src/gecko_panel.cpp`, `test/test_drive_mapping.cpp`
- Create: `ros2_ws/.gitignore` con `build/`, `install/`, `log/`

**Interfaces:**
- Consumes: `/gecko/state/controls` (Task 6).
- Produces:
  - Clase de plugin `gecko_rviz_panel/GeckoPanel`, derivada de `rviz_common::Panel`
  - `int gecko_rviz_panel::driveFromVector(double x, double y)`, con `x` a la derecha e `y` hacia arriba, ambos en `[-1, 1]`
  - Publica los 4 tópicos `/gecko/cmd/*` del spec

- [ ] **Step 1: Escribir la prueba del joystick**

```cpp
// test/test_drive_mapping.cpp
#include <gtest/gtest.h>
#include "gecko_rviz_panel/drive_mapping.hpp"
using gecko_rviz_panel::driveFromVector;

TEST(Drive, DeadZone) { EXPECT_EQ(driveFromVector(0, 0), 0); EXPECT_EQ(driveFromVector(0.1, 0.1), 0); }
TEST(Drive, Cardinals) {
  EXPECT_EQ(driveFromVector(0, 1), 1);  EXPECT_EQ(driveFromVector(0, -1), 2);
  EXPECT_EQ(driveFromVector(1, 0), 3);  EXPECT_EQ(driveFromVector(-1, 0), 4);
}
TEST(Drive, DominantAxis) { EXPECT_EQ(driveFromVector(0.7, 0.6), 3); EXPECT_EQ(driveFromVector(0.6, -0.7), 2); }
```

- [ ] **Step 2: Compilar y comprobar que falla**

Run: `cd IMU/gecko_imu/ros2_ws && ../tests/run_ros.sh colcon build --packages-select gecko_rviz_panel`
Expected: FAIL porque falta `drive_mapping.hpp`.

- [ ] **Step 3: Implementar `driveFromVector` y el panel**

Zona muerta: magnitud `< 0.2`. Controles:

- **`JoystickWidget`:** `QWidget` dibujado con `QPainter`. Emite `vectorChanged(double x, double y)` y vuelve al centro al soltar.
- **Palanca:** `QSlider` vertical, de `-45` a `45` grados, con botón `0°`. Publica en radianes al cambiar.
- **Perillas F y R:** `QDial`, de `-100` a `100`, con botón `0` y etiqueta de porcentaje. Publican `valor / 100.0`.
- **Publicación:** `drive` con un `QTimer` de 100 ms. El resto, solo por acción del usuario.
- **Estado inicial:** suscripción transient local a `/gecko/state/controls`, que actualiza los controles con `QSignalBlocker` para no republicar.

El nodo se obtiene con `getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node()`. Exportar con `PLUGINLIB_EXPORT_CLASS` y `pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)`.

- [ ] **Step 4: Compilar y ejecutar las pruebas**

Run: `cd IMU/gecko_imu/ros2_ws && ../tests/run_ros.sh colcon build --packages-select gecko_rviz_panel && ../tests/run_ros.sh colcon test --packages-select gecko_rviz_panel --event-handlers console_direct+ && ../tests/run_ros.sh colcon test-result --verbose`
Expected: compilación correcta y `0 failures`.

- [ ] **Step 5: Comprobar que RViz encuentra el plugin**

Run: `cd IMU/gecko_imu/ros2_ws && ../tests/run_ros.sh ros2 pkg prefix gecko_rviz_panel && test -f install/gecko_rviz_panel/share/ament_index/resource_index/rviz_common__pluginlib__plugin/gecko_rviz_panel && echo plugin_ok`
Expected: `plugin_ok`

---

### Task 9: Arranque completo y configuración de RViz

**Files:**
- Modify: `start_gecko_pc.sh`
- Create: `rviz/gecko.rviz`, `tests/unit/test_rviz_config.py`, `tests/integration/check_start_stop.sh`

**Interfaces:**
- Consumes: todo lo anterior.
- Produces: `start_gecko_pc.sh [start|stop]`, con las variables `GECKO_SKIP_HOTSPOT=1` y `GECKO_HEADLESS=1` (Gazebo con `-s` y RViz dentro de `xvfb-run -a`).

- [ ] **Step 1: Escribir las pruebas**

```python
# tests/unit/test_rviz_config.py
from pathlib import Path
import yaml

CONFIG = yaml.safe_load((Path(__file__).resolve().parents[2] / 'rviz/gecko.rviz').read_text())

def test_fixed_frame():
    assert CONFIG['Visualization Manager']['Global Options']['Fixed Frame'] == 'world'

def test_panel_is_loaded():
    assert 'gecko_rviz_panel/GeckoPanel' in [p['Class'] for p in CONFIG['Panels']]

def test_displays():
    displays = {d['Class']: d for d in CONFIG['Visualization Manager']['Displays']}
    assert displays['rviz_default_plugins/RobotModel']['Description Topic']['Value'] == '/robot_description'
    assert displays['rviz_default_plugins/Marker']['Topic']['Value'] == '/gecko/wall/marker'
    assert 'rviz_default_plugins/Grid' in displays
```

`check_start_stop.sh`, con `GECKO_SKIP_HOTSPOT=1 GECKO_HEADLESS=1` y **Conda sin limpiar** (Review Focus 5):

1. Ejecuta `start` y espera a que existan `/joint_states` y `/gecko/cmd/drive` con un publicador.
2. Comprueba que hay un proceso de cada: `gecko_twin_node.py`, `wall_climb_manager.py`, `udp_imu_receiver.py`, `imu_displacement_estimator.py`, `robot_state_publisher`, `rviz2`, y que los nodos Python usan `/usr/bin/python3`.
3. Ejecuta `start` otra vez y repite la comprobación de un proceso por nodo.
4. Ejecuta `stop` dos veces, ambas con código 0.
5. Comprueba que `pgrep` no encuentra ninguno de esos procesos ni `ign gazebo`.

- [ ] **Step 2: Ejecutar y comprobar que falla**

Run: `cd IMU/gecko_imu && /usr/bin/python3 -m pytest tests/unit/test_rviz_config.py -v`
Expected: FAIL porque `rviz/gecko.rviz` no existe.

- [ ] **Step 3: Implementar el arranque**

Aplicar la sección 11 del spec. El orden es: limpiar Conda, cargar ROS, hotspot (omitible), generar modelos, compilar el panel si `install/` falta o algún archivo de `src/` es más nuevo que `install/.gecko_build_stamp`, Gazebo, entidad, puente, receptor, gemelo, estimador, gestor, `robot_state_publisher` con `--params-file` y RViz.

Con `GECKO_SKIP_HOTSPOT=1`, `stop` tampoco toca el hotspot. Actualizar `require_command`, las comprobaciones de archivos, los `pkill` y la lista de nombres de `stop_all`.

- [ ] **Step 4: Ejecutar las pruebas**

Run: `cd IMU/gecko_imu && /usr/bin/python3 -m pytest tests/unit/test_rviz_config.py -v && bash tests/integration/check_start_stop.sh`
Expected: 3 passed y `start_stop_ok`.

---

### Task 10: Verificación de aceptación

**Files:**
- Create: `tests/integration/check_acceptance.py`

**Interfaces:**
- Consumes: `start_gecko_pc.sh` con `GECKO_SKIP_HOTSPOT=1 GECKO_HEADLESS=1`, y los tópicos del sistema.

- [ ] **Step 1: Implementar el verificador**

El script arranca el sistema, publica una IMU sintética en `/imu/data` (empezando nivelada, porque la primera muestra calibra el cero) y comandos en `/gecko/cmd/*`, y comprueba la pose real en Gazebo con `/world/empty/dynamic_pose/info` y los joints con `/world/empty/model/gecko/joint_state`. Imprime `PASS` o `FAIL` por criterio y siempre ejecuta `stop` al terminar.

| Criterio del spec | Comprobación |
|---|---|
| 2 | La entidad `gecko` existe con 9 links |
| 3 | Las 4 direcciones dan los signos de la tabla; adelante avanza `0.15 ± 0.03` m en 1 s |
| 4 | Servos en `0.5 ± 0.0175` rad, y siguen ahí 3 s después |
| 5 | Turbina F a `+30 ± 3` y R a `−60 ± 6` rad/s, sostenidas 3 s |
| 6 | Con pitch ±45° y avance, `abs(z − support(q, (0, 0, -1))) < 0.001` m |
| 7 | Con pitch −75°, `gecko_wall` aparece delante; `z` sube con adelante y se detiene en el límite |
| 8 | Con pitch 0°, la pared desaparece y `z` vuelve a `0.0284 ± 0.001` |
| 9 | Desviación de velocidad de ruedas menor a `1` rad/s durante 8 s |

- [ ] **Step 2: Ejecutar toda la batería**

Run: `cd IMU/gecko_imu && /usr/bin/python3 -m pytest tests/unit -v && tests/run_ros.sh /usr/bin/python3 -m pytest tests/integration -v && tests/run_ros.sh /usr/bin/python3 tests/integration/check_acceptance.py`
Expected: todas las pruebas pasan y cada criterio imprime `PASS`.

- [ ] **Step 3: Comprobar que no quedan procesos**

Run: `pgrep -af 'gecko_twin_node|wall_climb_manager|udp_imu_receiver|rviz2|ign gazebo' || echo limpio`
Expected: `limpio`

- [ ] **Step 4: Dejar constancia de lo que requiere al usuario**

Reportar que quedan dos comprobaciones manuales: el tacto de los controles en RViz con pantalla real, y la prueba con la Raspberry tras configurarle `Gecko-IMU` y `Gecko2026!`.
