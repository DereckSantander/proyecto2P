# Global Planner - Planificación Global con Dijkstra para Go2

Sistema de planificación de rutas global para el robot Go2 en ROS 2 usando el algoritmo de Dijkstra.

## 🧭 Resumen

Planificador global ligero escrito en Python para ROS 2 Humble que toma un mapa de ocupación, odometría del Go2 y un objetivo de RViz para generar un `nav_msgs/Path` con Dijkstra y publicarlo en `map`.

## 🚀 Ejecución Rápida

### Preparación Inicial (una sola vez)

```bash
cd ~/proyecto2p
colcon build --packages-select global_planner
source install/setup.bash
```

Coloca tu mapa en el directorio de mapas:
```bash
cp /ruta/a/tu/small_house.pgm ~/proyecto2p/src/global_planner/maps/map.pgm
```

---

## 📋 Pasos de Ejecución (5 Terminales)

### Terminal 1 — Map Server
Carga el mapa de ocupación:

```bash
source ~/proyecto2p/install/setup.bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$(cd ~/proyecto2p/src/global_planner/maps && pwd)/map.yaml"
```

Una vez que arranque, en otra pestaña de Terminal 1:
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

---

### Terminal 2 — Bringup del Go2
Lanza el robot con odometría, descripción y controladores:

```bash
source ~/proyecto2p/install/setup.bash
ros2 launch go2_config bringup.launch.py hardware_connected:=true
```

Este comando lanza automáticamente:
- `robot_state_publisher` (descripción del robot)
- `state_estimation_node` (estimación de pose)
- `footprint_to_odom_ekf` (publica `/odom`)

---

### Terminal 3 — Transformación Estática map→odom
(Solo necesario si tu robot no publica map→odom por localization/SLAM)

```bash
source ~/proyecto2p/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

---

### Terminal 4 — Planificador Dijkstra
Lanza el nodo de planificación:

```bash
source ~/proyecto2p/install/setup.bash
ros2 run global_planner dijkstra_planner
```

**Deberías ver en los logs:**
```
============================================================
✅ Algoritmo implementado
============================================================
✅ Mapa cargado correctamente
✅ Odometría recibida correctamente
📊 Estado: ✅ Mapa | ✅ Odometría | ❌ Objetivo
```

---

### Terminal 5 — RViz
Abre RViz (tu instancia manual):

```bash
source ~/proyecto2p/install/setup.bash
rviz2
```

**Configuración en RViz:**
1. Fixed Frame: `map`
2. Agrega display "OccupancyGrid" → Topic: `/map`
   - QoS → Reliability: **Reliable**
   - QoS → Durability: **Transient Local**
3. Agrega display "Path" → Topic: `/global_path`
4. Agrega display "RobotModel" → visualiza el Go2

---

## 🎮 Cómo Usar

1. Asegúrate de que todos los comandos están ejecutándose (5 terminales)
2. En RViz, haz clic en el botón **"2D Goal Pose"** (flecha verde superior)
3. Haz clic en cualquier punto del mapa para definir el objetivo
4. El planificador ejecutará Dijkstra automáticamente
5. Verás la trayectoria en verde en RViz

**Logs del planificador mostrarán:**
```
🎯 Nuevo objetivo: x=2.50, y=3.50
🔍 Ejecutando Dijkstra...
✅ CAMINO ENCONTRADO!
   - Nodos visitados: 1250
📤 Path publicado en /global_path
   - Puntos: 125
   - Longitud: 15.75m
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

## 🚀 Launch files

- `launch/map_server.launch.py`: despliega `nav2_map_server` con lifecycle manager y argumentos `map`/`use_sim_time`.
- `launch/planner_bringup.launch.py`: levanta mapa, lifecycle manager, nodo `dijkstra_planner` con parámetros por defecto y RViz configurado.
- `launch/planner_only.launch.py`: solo `dijkstra_planner` + RViz para entornos donde mapa/odom ya existen.

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

## ❌ Solución de Problemas

| Problema | Causa | Solución |
|----------|-------|----------|
| "Frame map does not exist" en RViz | Falta TF map→odom | Ejecuta Terminal 3 |
| "No map received" en RViz | QoS incorrecto en RViz | Cambia Durability a "Transient Local" |
| No aparece `/odom` | Go2 no lanzado | Verifica Terminal 2 |
| Objetivo no se recalcula | Planificador no corre | Verifica Terminal 4 logs |
| Path vacío o muy corto | Obstáculos bloquean todo | Reduce `inflation_radius` |

---

## 📝 Arquitectura del Sistema

```
Map Server (T1)
    ↓
    map_server → /map (OccupancyGrid)
                    ↓
                    ├─→ Planificador (T4)
                    │       ↓
                    │   Dijkstra Algorithm
                    │       ↓
                    │   /global_path
                    ↓ (visualizado en RViz)
RViz (T5)
    ↑
    ├─ 2D Goal Pose → /goal_pose
    ├─ Map Display ← /map
    └─ Path Display ← /global_path

Go2 Bringup (T2)
    ↓
    └─ state_estimation_node → /odom (Odometry)
                    ↓
                    Planificador (T4)
```

---