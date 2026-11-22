# Gecko: gemelo digital en Gazebo y RViz

Fecha: 2026-11-18. Estado: pendiente de aprobación.

## 1. Intención

El proyecto `mini_sumo_imu` muestra en Gazebo la orientación de un vehículo real, medida con una IMU BNO085 en una Raspberry Pi 5 y enviada por UDP. Ese sistema pasa a representar el vehículo oficial del proyecto, **Gecko**, un robot de inspección que trepa superficies.

Se busca un gemelo que combine dos fuentes:

- **La orientación del cuerpo** viene únicamente del vehículo real (IMU).
- **Los motores y el desplazamiento** vienen de un panel de control dentro de RViz.

## 2. Objetivos

1. Sustituir el modelo del mini sumo en Gazebo por el vehículo de `Micro2_URDFV2`.
2. Renombrar todo lo que diga "sumo" a "Gecko", incluido el ejecutable `.sh`.
3. Crear un panel en RViz con joystick, palanca y dos perillas que controle los 8 motores del gemelo.
4. Hacer que el Gecko se desplace por el piso y trepe la pared sin volar ni atravesar superficies.

## 3. Fuera de alcance

- **El panel no comanda el vehículo real.** Solo actúa sobre la simulación. No se envía nada a la Raspberry.
- No se simula empuje ni adherencia de las turbinas. Su giro es visual.
- No se usa la posición estimada por la IMU. El desplazamiento sale solo del joystick.
- No se modifica el código de la Raspberry, que no está en este PC.
- No se modifica el URDF original ni el proyecto `Macroproyecto`.

## 4. Enfoques considerados

| Enfoque | Resultado |
|---|---|
| **Panel propio de RViz en C++** | **Elegido por el usuario.** Es el único con los cuatro controles dentro de RViz. |
| Marcadores interactivos en la vista 3D | Descartado: no se comportan como joystick ni perilla. |
| Ventana aparte en Python | Descartado: deja los controles fuera de RViz. |

## 5. Decisiones del usuario

- El Gecko **se desplaza** en Gazebo con el joystick.
- Al avanzar inclinado **no vuela ni atraviesa el piso**.
- Con la pared activa, el Gecko **la trepa**.
- El panel va **acoplado dentro de RViz**.
- La **palanca conserva su posición** al soltarla.
- Las **perillas conservan su valor** y admiten valores negativos, que invierten el giro.

## 6. Renombrado

| Antes | Después |
|---|---|
| `IMU/mini_sumo_imu/` | `IMU/gecko_imu/` |
| `start_mini_sumo_pc.sh` | `start_gecko_pc.sh` |
| Entidad `mini_sumo` | `gecko` |
| Pared `mini_sumo_wall` | `gecko_wall` |
| `~/.local/state/mini_sumo_imu` | `~/.local/state/gecko_imu` |
| Hotspot `MiniSumo-IMU` | `Gecko-IMU` |
| Clave `MiniSumo2026!` | `Gecko2026!` |
| Textos "Mini Sumo" en scripts y comentarios | "Gecko" |

**Consecuencia:** la Raspberry debe configurarse con la red y clave nuevas para volver a enviar datos.

Se conservan sin cambios:

- `IMU/mini_sumo_imu.zip`, como respaldo histórico del estado anterior.
- El perfil antiguo `MiniSumo-IMU` de NetworkManager. No se borra.
- El nombre del paquete `Micro2_URDFV2`.

## 7. Arquitectura

```
Raspberry ──UDP 5005──> udp_imu_receiver ──/imu/data──┬─> imu_displacement_estimator
                                                      │        │ /imu/estimated_pitch_deg
                                                      │        v
Panel RViz ──/gecko/cmd/*──> gecko_twin_node <──wall──> wall_climb_manager ──> Gazebo (pared)
                                  │
             ┌────────────────────┼───────────────────┐
             v                    v                   v
      set_pose (Gazebo)   comandos de joints   /joint_states + TF ──> RViz
                          (ros_gz_bridge)
```

`gecko_twin_node.py` es la **única autoridad** sobre la pose y los joints. Evoluciona desde `imu_to_gazebo.py` y conserva su calibración, el servicio `/imu/calibrate` y los parámetros de montaje de la IMU.

### Tópicos

| Tópico | Tipo | Origen → destino |
|---|---|---|
| `/gecko/cmd/drive` | `std_msgs/Int8` (0 alto, 1 adelante, 2 atrás, 3 derecha, 4 izquierda) | Panel → gemelo, a 10 Hz |
| `/gecko/cmd/servo_angle` | `std_msgs/Float64` (rad) | Panel → gemelo |
| `/gecko/cmd/turbine_f`, `/gecko/cmd/turbine_r` | `std_msgs/Float64` (−1 a 1) | Panel → gemelo |
| `/gecko/wall/target_pose` | `geometry_msgs/PoseStamped` | Gemelo → gestor de pared |
| `/gecko/wall/active` | `std_msgs/Bool` (transient local) | Gestor de pared → gemelo |
| `/gecko/wall/pose` | `geometry_msgs/PoseStamped` (transient local) | Gestor de pared → gemelo |
| `/gecko/wall/marker` | `visualization_msgs/Marker` | Gestor de pared → RViz |
| `/joint_states` | `sensor_msgs/JointState` | Gemelo → `robot_state_publisher` |

## 8. Panel de control

Paquete C++ `gecko_rviz_panel` (`ament_cmake`, `pluginlib`, `rviz_common`, Qt5), en `IMU/gecko_imu/ros2_ws/src/`.

| Control | Comportamiento |
|---|---|
| **Joystick** | Cuatro direcciones a velocidad constante. Zona muerta del 20 %. En diagonal gana el eje dominante. Al soltarlo vuelve al centro y las ruedas paran. |
| **Palanca** | Un solo valor para `Turbine_Servo_F_Joint` y `Turbine_Servo_R_Joint`, de −45° a +45°. Hacia adelante es positivo. Conserva la posición al soltarla. Tiene un botón `0°`. |
| **Perilla F** | Controla `Turbine_F_Joint` de −100 % a +100 %. Conserva el valor al soltarla. Tiene un botón `0`. |
| **Perilla R** | Igual que la anterior, para `Turbine_R_Joint`. |

### Mando de las ruedas

| Dirección | FL | RL | FR | RR |
|---|---|---|---|---|
| Adelante | + | + | + | + |
| Atrás | − | − | − | − |
| Derecha | + | + | − | − |
| Izquierda | − | − | + | + |

Los cuatro ejes apuntan a +Y del vehículo, por lo que un valor positivo lo impulsa hacia +X, que es el frente.

### Valores iniciales (parámetros ajustables)

- Velocidad lineal: `0,15 m/s`. Con el radio de rueda de `0,015 m` equivale a `10 rad/s`.
- Velocidad máxima de turbina: `60 rad/s`.

## 9. Movimiento y contacto

El ciclo corre a 30 Hz con `dt` medido y limitado a `0,1 s`. La orientación `R` es siempre la de la IMU calibrada. Antes de recibir la primera muestra se usa orientación nivelada, para poder probar los controles sin la Raspberry.

### Función de apoyo

`S(u)` es la distancia máxima que alcanza el vehículo desde `Base_link` en la dirección unitaria `u`, con la envolvente:

- **Ruedas:** discos de radio `0,015 m`, eje Y del cuerpo, centrados en los orígenes de sus joints.
- **Chasis:** caja con `x = ±0,153`, `y = ±0,1308`, `z` de `−0,0209` a `0,068`.
- **Servos y turbinas:** discos de radio `0,056 m`, eje Y del cuerpo, en `x = ±0,09`, `y = ±0,0672`, `z = 0,0415`. Cubren todo el recorrido de ±45°.

Para un disco de centro `c` y eje `a`, el aporte es `(R c)·u + r·sqrt(1 − ((R a)·u)²)`.

### Estados

**PISO**

- El avance sigue `f`, la proyección horizontal normalizada de `R x̂`.
- Si esa proyección mide menos de `0,1`, se usa el último rumbo válido.
- La altura es siempre `z = S(−Ẑ)`, de modo que el punto más bajo toca el piso.
- Izquierda y derecha solo hacen girar las ruedas.

**PARED** (activo mientras `/gecko/wall/active` sea verdadero)

- `h` es el rumbo horizontal fijado al crear la pared. `F` es un punto de su cara. `l = Ẑ × h`.
- La distancia a la pared se fija con `p·h = F·h − S(h)`.
- El avance sigue la proyección normalizada de `R x̂` sobre el plano de la pared. Adelante sube y atrás baja.
- Límites: `z ≥ S(−Ẑ)`, `z ≤ 2,0 − S(Ẑ)` y lateral dentro de los 2 m de ancho.

**DESCENSO** (al desactivarse la pared)

- Baja a `0,5 m/s` hasta `z = S(−Ẑ)` y pasa a PISO. Ignora el joystick mientras dura.

### Pared

Se conservan los umbrales actuales: aparece con pitch ≤ −60° durante 0,3 s y desaparece con pitch ≥ −50° durante 0,5 s.

El gemelo publica continuamente `target_pose`. Su cara queda en `F = p + h·(S(h) + 0,01)`, su centro en `F + h·0,05`, a `z = 1,0 m`, con guiñada `atan2(h.y, h.x)`. El gestor usa esa pose al crear la pared.

### Seguridad de los comandos

- Si no llega `/gecko/cmd/drive` durante `0,5 s`, por ejemplo al cerrar RViz, las ruedas se detienen.
- Servos y turbinas mantienen su último valor.

## 10. Modelos generados

`generate_gecko_models.py` lee el URDF original al arrancar y produce en `models/gecko/`:

- **`gecko_rviz.urdf`:** con rutas absolutas `file://` a las mallas, porque el paquete original es de ROS 1 y RViz 2 no lo resuelve.
- **`gecko_gazebo.urdf`:** además, sin colisiones, sin gravedad y con los plugins `JointController` (6 joints de velocidad) y `JointPositionController` (2 servos).

Se quitan las colisiones porque el contacto se resuelve en la sección 9 y `Base_link.STL` tiene 283 624 triángulos.

## 11. Arranque

`start_gecko_pc.sh [start|stop]` conserva el flujo actual y añade:

1. Limpiar el entorno de Conda. `python3` apunta hoy a Miniconda 3.14, donde `rclpy` no funciona, así que los nodos usarán `/usr/bin/python3`.
2. Generar los modelos.
3. Compilar el panel con `colcon` si falta o cambió.
4. Ampliar el puente con los 8 comandos de joints.
5. Lanzar `gecko_twin_node`, `robot_state_publisher` y RViz con `gecko.rviz`.
6. Respetar `GECKO_SKIP_HOTSPOT=1`, para probar sin apagar el Wi-Fi.

`stop` debe cerrar también los procesos nuevos.

## 12. Criterios de aceptación

1. No quedan menciones a "sumo" en `IMU/gecko_imu/`.
2. Gazebo y RViz muestran el Gecko completo con sus 9 mallas.
3. Cada dirección del joystick produce los signos de la tabla de ruedas y el desplazamiento esperado.
4. La palanca mueve ambos servos juntos, respeta ±45° y conserva su posición.
5. Cada perilla controla solo su turbina, invierte el giro con valores negativos y mantiene la velocidad al soltarla.
6. Con pitch y roll de hasta ±90°, ningún punto de la envolvente queda bajo el piso y el punto más bajo queda a menos de 1 mm de él mientras avanza.
7. Con pitch de −75° aparece la pared delante del Gecko, sube y baja con el joystick, y se detiene en los límites.
8. Al volver el pitch, la pared desaparece y el Gecko baja hasta el piso.
9. `set_pose` no produce vibración visible en los joints.
10. `start_gecko_pc.sh stop` no deja procesos del sistema activos.

Las pruebas se hacen con IMU sintética. La prueba con la Raspberry real queda a cargo del usuario, tras actualizar su Wi-Fi.

## 13. Riesgos

- **`set_pose` y joints simultáneos.** Fortress podría alterar el giro de los joints al recolocar el modelo a 30 Hz. Se comprueba primero con un prototipo mínimo. Si falla, la alternativa es mover los joints sin dinámica.
- **Dos calibraciones.** El estimador de pitch y el gemelo calibran por separado, como ya ocurre hoy. Llamar a `/imu/calibrate` solo afecta al gemelo, lo que puede desplazar el umbral de la pared.

## 14. A criterio de quien implemente

- Aspecto visual del panel.
- Ganancias de los controladores de servos, con error final menor a 1°.
- Organización interna del código del gemelo y sus pruebas.
