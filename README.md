# Global Planner - Planificación Global con Dijkstra para Go2

Sistema de planificación de rutas global para el robot Go2 en ROS 2 usando el algoritmo de Dijkstra.

## YouTube

▶️ [Demo del proyecto en YouTube](https://www.youtube.com/watch?v=E58xNA6WzRI)

## 🧭 Resumen

Planificador global ligero escrito en Python para ROS 2 Humble que toma un mapa de ocupación, odometría del Go2 y un objetivo de RViz para generar un `nav_msgs/Path` con Dijkstra y publicarlo en `map`.

## � Dependencias

- ROS 2 Humble (tested)
- `rclpy`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `tf2_ros`, `tf2_geometry_msgs`
- `nav2_map_server`, `nav2_lifecycle_manager`
- `gazebo_ros`, `rviz2`
- Paquetes Go2: `go2_config`, `go2_description` (del workspace `unitree-go2-ros2`)

## 🔧 Instalación y Compilación

### Preparación Inicial (una sola vez)

```bash
cd ~/proyecto2p
colcon build 
source install/setup.bash
```

Coloca tu mapa en el directorio de mapas:
```bash
cp /ruta/a/tu/small_house.pgm ~/proyecto2p/src/global_planner/maps/map.pgm
```

---

## 🚀 Ejecución del Sistema

Ejecuta cada componente en su propia terminal:

#### Terminal 1 — Gazebo (Mundo Small House + Robot Go2)
```bash
cd ~/proyecto2p
source install/setup.bash
ros2 launch go2_config gazebo_velodyne.launch.py world:=small_house
```
*Lanza Gazebo con el mundo small_house y spawnea el robot Go2*

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

#### Terminal 3 — Planificador Global (Dijkstra)
```bash
cd ~/proyecto2p
source install/setup.bash
ros2 launch global_planner planner.launch.py
```
*Escucha `/map`, `/odom` y `/goal_pose`; publica `/global_path`*

#### Terminal 4 — Controlador PID Local
```bash
cd ~/proyecto2p
source install/setup.bash
ros2 launch go2_local_controller controller.launch.py
```
*Convierte `/global_path` y `/odom` en `/cmd_vel` para seguir la ruta*

#### Terminal 5 — TF Estático map→odom
*Publica la transformación estática entre `map` y `odom`. Solo ejecuta si tu sistema no lo hace automáticamente:*
```bash
cd ~/proyecto2p
source install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```
**Nota:** Los argumentos `0 0 0 0 0 0` representan `x y z roll pitch yaw` (origen coincidente). Ajusta si el robot no empieza en el origen del mapa.

#### Terminal 6 — RViz (Visualización)
```bash
rviz2
```

**Configuración en RViz:**
1. Fixed Frame: `map`
2. Display "OccupancyGrid" → Topic: `/map` (QoS: Reliable + Transient Local)
3. Display "Path" → Topic: `/global_path`
4. Display "RobotModel"
5. Usa **"2D Goal Pose"** para enviar objetivos

---

## 🎮 Uso del Sistema

1. **Verifica que todos los nodos estén activos:**
   ```bash
   ros2 node list
   # Deberías ver: /map_server, /dijkstra_planner, /pid_controller, /gazebo, etc.
   ```

2. **En RViz, define el objetivo:**
   - Haz clic en **"2D Goal Pose"** (flecha verde en barra superior)
   - Haz clic en un punto válido del mapa
   - Arrastra para definir la orientación

3. **Observa la navegación autónoma:**
   - El planificador calculará el camino (verde en RViz)
   - El controlador PID moverá el robot siguiendo la ruta
   - El robot se detendrá al alcanzar el objetivo

**Logs esperados del planificador:**
```
🎯 Nuevo objetivo: x=2.50, y=3.50
🔍 Ejecutando Dijkstra...
✅ CAMINO ENCONTRADO!
   - Nodos visitados: 1250
📤 Path publicado en /global_path
   - Puntos: 125
   - Longitud: 15.75m
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
- `/odom` — Odometría del Go2
- `/goal_pose` — Objetivo desde RViz
- `/map` — Mapa de ocupación

### Publicaciones
- `/global_path` — Trayectoria (nav_msgs/Path)

---

## 🧠 Algoritmo usado

- **Dijkstra sobre grilla 2D**: convierte `map` (OccupancyGrid) a matriz, infla obstáculos (`inflation_radius`) y calcula vecinos 4 u 8 direcciones (`use_diagonal`).
- **Costos y visitados**: heap de prioridad (`heapq`) con costo acumulado; rechaza celdas ocupadas `occupied_threshold` y desconocidas.
- **Parámetros clave**: `use_diagonal`, `occupied_threshold`, `inflation_radius`, `path_resolution` (controla densidad del `Path`).
- **Extras**: suavizado básico (omite puntos colineales), QoS `TRANSIENT_LOCAL` para recibir mapas previos y chequeo periódico de estado de datos requeridos.

---

## 🤖 Control Local PID (seguimiento de ruta)

- **Objetivo**: convertir `Path` + `Odometry` en comandos de velocidad `cmd_vel` para navegación autónoma básica en el simulador Unitree Go2.
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

- `lookahead_distance`: distancia objetivo hacia adelante en la ruta (m).
- `goal_tolerance`, `yaw_tolerance`: tolerancias de llegada (m, rad).
- `lin_kp`, `lin_ki`, `lin_kd`: ganancias lineales.
- `ang_kp`, `ang_ki`, `ang_kd`: ganancias angulares.
- `max_linear_speed`, `max_angular_speed`: límites de velocidad.

## 🚀 Launch Files Disponibles

### Paquete `global_planner`
- **`planner.launch.py`**: Planificador global Dijkstra
- **`map_server.launch.py`**: Map Server con lifecycle manager

### Paquete `go2_local_controller`
- **`controller.launch.py`**: Controlador PID local (requiere `/odom` y `/global_path` activos)

**Nota:** Todos los launches usan `use_sim_time:=true` por defecto para sincronización con Gazebo.

---

## ⚙️ Parámetros

Lanza el planificador con parámetros personalizados:

```bash
ros2 run global_planner dijkstra_planner --ros-args \
  -p use_diagonal:=true \
  -p occupied_threshold:=65 \
  -p inflation_radius:=0.3 \
  -p path_resolution:=0.05
```

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `use_diagonal` | true/false | Permite movimientos diagonales |
| `occupied_threshold` | 0-100 | Umbral de ocupación |
| `inflation_radius` | metros | Radio de seguridad alrededor de obstáculos |
| `path_resolution` | metros | Resolución del camino generado |

---

## 🔍 Verificación y Debugging

### Verificar tópicos activos:
```bash
# Ver todos los tópicos
ros2 topic list

# Verificar mapa
ros2 topic echo /map --once

# Verificar odometría
ros2 topic hz /odom
ros2 topic echo /odom --once

# Ver path calculado
ros2 topic echo /global_path

# Ver comandos de velocidad
ros2 topic echo /cmd_vel
```

### Verificar nodos:
```bash
# Listar nodos activos
ros2 node list

# Ver info de un nodo
ros2 node info /dijkstra_planner
ros2 node info /pid_controller
```

### Verificar transformaciones (TF):
```bash
# Ver árbol de transformaciones
ros2 run tf2_tools view_frames

# Verificar TF específica
ros2 run tf2_ros tf2_echo map odom
```

## 📝 Arquitectura del Sistema

```
                    ┌─────────────────┐
                    │   Gazebo World  │
                    │  (small_house)  │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │   Go2 Robot     │
                    │  (simulation)   │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  /odom, /tf     │
                    └─────┬─────┬─────┘
                          │     │
         ┌────────────────┘     └──────────────┐
         │                                     │
    ┌────▼──────┐                      ┌──────▼─────┐
    │ Map Server│                      │  Dijkstra  │
    │           │                      │  Planner   │
    │ /map      │─────────────────────▶│            │
    └───────────┘                      └──────┬─────┘
                                              │
                                       ┌──────▼─────┐
                                       │/global_path│
                                       └──────┬─────┘
                                              │
                          ┌───────────────────┴──────┐
                          │                          │
                    ┌─────▼──────┐            ┌──────▼─────┐
                    │    RViz    │            │    PID     │
                    │            │            │ Controller │
                    │ Visualiza  │            │            │
                    │ y Goal     │            └──────┬─────┘
                    └────────────┘                   │
                                              ┌──────▼─────┐
                                              │  /cmd_vel  │
                                              └──────┬─────┘
                                                     │
                                              ┌──────▼─────┐
                                              │   Go2      │
                                              │  Movement  │
                                              └────────────┘
```

**Flujo de datos:**
1. **Gazebo** simula el mundo y el robot Go2
2. **Go2 Bringup** publica `/odom` (pose actual)
3. **Map Server** carga y publica `/map` (OccupancyGrid)
4. **Usuario** define objetivo en **RViz** → `/goal_pose`
5. **Planificador** recibe `/map`, `/odom`, `/goal_pose` → calcula y publica `/global_path`
6. **Controlador PID** recibe `/global_path` y `/odom` → calcula y publica `/cmd_vel`
7. **Simulador** mueve el robot según `/cmd_vel`
8. **RViz** visualiza todo el proceso

---

## 🗺️ Grafo de nodos ROS 2

```
/gazebo
/map_server ──/map──────────────────────────────▶ /dijkstra_planner
/state_estimation_node ──/odom──────────────────▶ /dijkstra_planner
                                                  /pid_controller
RViz ──/goal_pose──────────────────────────────▶ /dijkstra_planner
/dijkstra_planner ──/global_path───────────────▶ /pid_controller
                                                  RViz
/pid_controller ──/cmd_vel─────────────────────▶ /gazebo (Go2)
```

---
