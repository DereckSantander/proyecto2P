# Global Planner - Planificación Global con Dijkstra para TurtleBot3

Sistema de planificación de rutas global para el robot TurtleBot3 en ROS 2 usando el algoritmo de Dijkstra.

## 🧭 Resumen

Planificador global ligero escrito en Python para ROS 2 Humble que toma un mapa de ocupación, odometría del TurtleBot3 y un objetivo de RViz para generar un `nav_msgs/Path` con Dijkstra y publicarlo en `map`.

## 📦 Dependencias

- ROS 2 Humble (tested)
- `rclpy`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `tf2_ros`, `tf2_geometry_msgs`
- `nav2_map_server`, `nav2_lifecycle_manager`
- `gazebo_ros`, `rviz2`
- TurtleBot3 packages:
  ```bash
  sudo apt install ros-humble-turtlebot3* ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2
  ```
- Este workspace: `global_planner`, `go2_local_controller`

## 🔧 Instalación y Compilación

### Preparación Inicial (una sola vez)

```bash
cd ~/proyecto2p
colcon build 
source install/setup.bash
```

Asegúrate de que TurtleBot3 esté instalado:
```bash
sudo apt update
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2
```

---

## 🚀 Ejecución del Sistema

Ejecuta cada componente en su propia terminal para control granular:

#### Terminal 1 — Gazebo (Mundo Small House + Robot TurtleBot3)
```bash
cd ~/proyecto2p
source install/setup.bash
export TURTLEBOT3_MODEL=burger   # o waffle_pi
ros2 launch global_planner turtlebot3_sim.launch.py
```
*Lanza Gazebo con el mundo small_house y spawnea el TurtleBot3*

#### Terminal 2 — Map Server
```bash
cd ~/proyecto2p
source install/setup.bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$(cd ~/proyecto2p/src/global_planner/maps && pwd)/map.yaml" \
  -p use_sim_time:=true
```
*En otra pestaña del mismo terminal, configura y activa el lifecycle:*
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```
*Carga y publica el mapa de ocupación en `/map`*

#### Terminal 3 — TF Estático map→odom
*Publica la transformación estática entre `map` y `odom`:*
```bash
cd ~/proyecto2p
source install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
**Nota:** Los argumentos `0 0 0 0 0 0` representan `x y z roll pitch yaw` (origen coincidente). Ajusta si el robot no empieza en el origen del mapa.

#### Terminal 4 — Planificador Global (Dijkstra)
```bash
cd ~/proyecto2p
source install/setup.bash
ros2 launch global_planner planner.launch.py
```
*Escucha `/map`, `/odom` y `/goal_pose`; publica `/global_path`*

#### Terminal 5 — Controlador PID Local
```bash
cd ~/proyecto2p
source install/setup.bash
ros2 launch go2_local_controller controller.launch.py
```
*Convierte `/global_path` y `/odom` en `/cmd_vel` para seguir la ruta*

#### Terminal 6 — RViz (Visualización)
```bash
rviz2
```

**Configuración en RViz:**
1. Fixed Frame: `map`
2. Display "OccupancyGrid" → Topic: `/map` (QoS: Reliable + Transient Local)
3. Display "Path" → Topic: `/global_path`
4. Display "RobotModel"
5. Display "TF" para ver la transformación tree
6. Usa **"2D Goal Pose"** para enviar objetivos

---

## 🎮 Uso del Sistema

1. **Verifica que todos los nodos estén activos:**
   ```bash
   ros2 node list
   # Deberías ver: /map_server, /dijkstra_planner, /pid_controller, /gazebo, /gazebo_ros_..., etc.
   ```

2. **Verifica que la odometría funciona:**
   ```bash
   ros2 topic echo /odom --once
   ```
   Deberías ver la pose inicial cerca de (0, 0, 0).

3. **En RViz, define el objetivo:**
   - Haz clic en **"2D Goal Pose"** (flecha verde en barra superior)
   - Haz clic en un punto válido del mapa
   - Arrastra para definir la orientación deseada

4. **Observa la navegación autónoma:**
   - El planificador calculará el camino (verde en RViz)
   - El controlador PID moverá el robot siguiendo la ruta
   - El robot se detendrá al alcanzar el objetivo
   - Observa en Gazebo cómo se mueve el TurtleBot3

**Logs esperados del planificador:**
```
🎯 Nuevo objetivo: x=1.50, y=0.80
🔍 Ejecutando Dijkstra...
✅ CAMINO ENCONTRADO!
   - Nodos visitados: 850
📤 Path publicado en /global_path
   - Puntos: 95
   - Longitud: 8.25m
```

**Logs esperados del controlador:**
```
✅ PID Controller iniciado
[Movimiento hacia objetivo...]
[Objetivo alcanzado - robot detenido]
```

---

## 📊 Tópicos

### Suscripciones
- `/odom` — Odometría del TurtleBot3
- `/goal_pose` — Objetivo desde RViz
- `/map` — Mapa de ocupación

### Publicaciones
- `/global_path` — Trayectoria (nav_msgs/Path)
- `/cmd_vel` — Comandos de velocidad (geometry_msgs/Twist)

### TF (Transformaciones)
- `map` → `odom` — Transformación estática (fija)
- `odom` → `base_footprint` — Odometría del robot (dinámica)

---

## 🧠 Algoritmo usado

- **Dijkstra sobre grilla 2D**: convierte `map` (OccupancyGrid) a matriz, infla obstáculos (`inflation_radius`) y calcula vecinos 4 u 8 direcciones (`use_diagonal`).
- **Costos y visitados**: heap de prioridad (`heapq`) con costo acumulado; rechaza celdas ocupadas `occupied_threshold` y desconocidas.
- **Parámetros clave**: `use_diagonal`, `occupied_threshold`, `inflation_radius`, `path_resolution` (controla densidad del `Path`).
- **Extras**: suavizado básico (omite puntos colineales), QoS `TRANSIENT_LOCAL` para recibir mapas previos y chequeo periódico de estado de datos requeridos.

---

## 🤖 Control Local PID (seguimiento de ruta)

- **Objetivo**: convertir `Path` + `Odometry` en comandos de velocidad `cmd_vel` para navegación autónoma básica.
- **Entradas**: `/global_path` (nav_msgs/Path), `/odom` (nav_msgs/Odometry).
- **Salida**: `/cmd_vel` (geometry_msgs/Twist) con `linear.x` y `angular.z`.
- **Selección de waypoint**: se elige el punto de la ruta más cercano al robot y se avanza hasta alcanzar una distancia acumulada de `lookahead_distance`; si no se alcanza, se toma el último punto.
- **Errores**: posición `dist_err = hypot(tx-cx, ty-cy)` y orientación `yaw_err = atan2(ty-cy, tx-cx) - yaw_actual` (normalizado a ±π).
- **PID**:
   - Lineal: `v = Kp*d + Ki*∫d + Kd*Δd/Δt`.
   - Angular: `w = Kp*e_yaw + Ki*∫e_yaw + Kd*Δe_yaw/Δt`.
   - Limitadores: `|w| ≤ max_angular_speed`, `0 ≤ v ≤ max_linear_speed*(1 - |w|/max_angular_speed)` para suavizar giros.
- **Detención**: si `dist_err < goal_tolerance` y `|yaw_err| < yaw_tolerance` → publicar `0,0`.


---

## 🔌 Parámetros del controlador

- `lookahead_distance`: distancia objetivo hacia adelante en la ruta (m). Recomendado: 0.3-0.5 m para TurtleBot3.
- `goal_tolerance`: tolerancia de distancia al objetivo (m). Recomendado: 0.1-0.2 m.
- `yaw_tolerance`: tolerancia de orientación al objetivo (rad). Recomendado: 0.1-0.3 rad.
- `lin_kp`, `lin_ki`, `lin_kd`: ganancias lineales. Recomendado: Kp=1.0, Ki=0.1, Kd=0.5.
- `ang_kp`, `ang_ki`, `ang_kd`: ganancias angulares. Recomendado: Kp=1.5, Ki=0.05, Kd=0.3.
- `max_linear_speed`: velocidad lineal máxima (m/s). Recomendado: 0.22 m/s (límite seguro de TurtleBot3 burger).
- `max_angular_speed`: velocidad angular máxima (rad/s). Recomendado: 2.84 rad/s.

Ejemplo con parámetros ajustados para TurtleBot3:

```bash
ros2 run go2_local_controller pid_controller --ros-args \
   -p lookahead_distance:=0.4 \
   -p goal_tolerance:=0.15 \
   -p yaw_tolerance:=0.2 \
   -p lin_kp:=1.0 \
   -p lin_ki:=0.1 \
   -p lin_kd:=0.5 \
   -p ang_kp:=1.5 \
   -p ang_ki:=0.05 \
   -p ang_kd:=0.3 \
   -p max_linear_speed:=0.22 \
   -p max_angular_speed:=2.84
```

---

## 📝 Notas Importantes

- **TurtleBot3 burger vs waffle_pi**: El burger es más lento pero suficiente para pruebas. Ajusta `max_linear_speed` según el modelo.
- **Resolución del mapa**: Un mapa de baja resolución (0.05 m/px) funciona bien para TurtleBot3.
- **Mundo**: `turtlebot3_world.launch.py` carga un mundo predefinido. Personalízalo según necesites.
- **Control PID**: Los parámetros iniciales son sugerencias. Ajusta según tu mapa y comportamiento deseado.

---

## 🎓 Recursos

- [TurtleBot3 Documentation](https://docs.turtlebot.com/)
- [ROS 2 Navigation](https://docs.nav2.org/)
- [Dijkstra Algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm)

