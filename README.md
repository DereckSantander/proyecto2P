# Global Planner - Planificación Global de Trayectoria

## 📝 Descripción del Proyecto

Paquete ROS 2 que implementa un sistema de **planificación global de trayectoria** para robots móviles utilizando el **algoritmo de Dijkstra**. El sistema permite al robot calcular trayectorias óptimas desde su posición actual hasta un objetivo definido por el usuario en RViz, considerando obstáculos del entorno y garantizando caminos libres de colisiones mediante inflación de obstáculos.

**Funcionalidades:**
- Planificación global con algoritmo de Dijkstra
- Recepción de pose del robot desde odometría (`/odom`)
- Definición de objetivos mediante RViz ("2D Goal Pose")
- Procesamiento de mapas de ocupación (`OccupancyGrid`)
- Publicación de trayectorias (`nav_msgs/Path`)
- Recalculación automática ante nuevos objetivos
- Inflación de obstáculos para seguridad del robot

---

## 🧮 Algoritmo Implementado: Dijkstra

### Descripción del algoritmo

El **algoritmo de Dijkstra** es un método de búsqueda de caminos que encuentra la ruta de menor costo desde un nodo inicial hasta un nodo objetivo en un grafo ponderado. A diferencia de A*, Dijkstra **no utiliza heurística** (h(n) = 0), lo que garantiza encontrar siempre el camino óptimo explorando uniformemente en todas direcciones.

### Funcionamiento

1. **Inicialización**:
   - Se crea un conjunto `open_set` (cola de prioridad) con el nodo inicial
   - `g_score[start] = 0` (costo acumulado desde el inicio)
   - `f_score[start] = 0` (en Dijkstra, f = g, sin heurística)

2. **Iteración principal**:
   ```
   Mientras open_set no esté vacío:
       - Extraer nodo con menor f_score (costo acumulado)
       - Si es el objetivo: reconstruir y devolver trayectoria
       - Para cada vecino del nodo actual:
           * Calcular nuevo costo: g_nuevo = g_actual + costo_movimiento
           * Si g_nuevo < g_vecino (o vecino no visitado):
               - Actualizar g_score[vecino] = g_nuevo
               - Actualizar f_score[vecino] = g_nuevo (sin heurística)
               - Añadir vecino a open_set
   ```

3. **Reconstrucción de trayectoria**:
   - Se sigue el diccionario `came_from` desde el objetivo hasta el inicio
   - Se convierte de coordenadas de grid a coordenadas del mundo

### Variables principales

| Variable | Tipo | Descripción |
|----------|------|-------------|
| `open_set` | heap (priority queue) | Nodos por explorar, ordenados por f_score |
| `came_from` | dict | Almacena el nodo predecesor de cada nodo visitado |
| `g_score` | dict | Costo acumulado desde el inicio hasta cada nodo |
| `f_score` | dict | En Dijkstra: f = g (sin heurística) |
| `grid` | numpy array 2D | Matriz del mapa (0-100: libre-ocupado) |
| `resolution` | float | Metros por celda del grid |
| `inflation_radius` | float | Radio de inflación de obstáculos en metros |

### Costos de movimiento

El algoritmo permite movimiento en **8 direcciones** con costos diferenciados:

| Dirección | Costo |
|-----------|-------|
| Cardinal (↑ ↓ ← →) | 1.0 |
| Diagonal (↗ ↘ ↙ ↖) | 1.414 (√2) |

### Modificaciones implementadas

1. **Inflación de obstáculos**:
   - Pre-procesamiento del mapa para expandar obstáculos
   - Radio configurable vía parámetro `inflation_radius` (default: 0.3m)
   - Considera el tamaño físico del robot para evitar colisiones

2. **Conversión de coordenadas**:
   - Funciones `world_to_grid()` y `grid_to_world()`
   - Transformación bidireccional entre sistema de coordenadas del mundo y celdas del grid
   - Permite trabajar con coordenadas reales del robot

3. **Validación de celdas**:
   - Verificación de límites del mapa antes de explorar
   - Comprobación de ocupación (umbral: 50% de probabilidad)
   - Evita planificar sobre obstáculos o fuera del mapa

4. **Optimización de búsqueda**:
   - Uso de `heapq` para cola de prioridad eficiente (O(log n) por operación)
   - Almacenamiento sparse de costos (solo nodos visitados)
   - Reducción de memoria y tiempo de cómputo

### Complejidad computacional

- **Temporal**: O(E log V) donde E = número de aristas, V = número de vértices
- **Espacial**: O(V) para almacenar scores y predecesores

### Diferencia con A*

| Aspecto | Dijkstra (Implementado) | A* |
|---------|------------------------|-----|
| Heurística | h(n) = 0 | h(n) = distancia_euclidiana |
| Exploración | Uniforme en todas direcciones | Dirigida hacia objetivo |
| Optimalidad | **Siempre garantizada** | Garantizada si h es admisible |
| Velocidad | Más lento (explora más nodos) | Más rápido |
| Uso recomendado | Mapas pequeños/medianos | Mapas grandes |

---

## 📊 Estructura del Paquete ROS 2

### ROS Node Graph

```
┌─────────────────────────────────────────────────────────────────┐
│                      SISTEMA DE PLANIFICACIÓN                    │
└─────────────────────────────────────────────────────────────────┘

         ┌─────────────────┐
         │   map_server    │
         │  (nav2_map_server)
         └────────┬────────┘
                  │ /map (OccupancyGrid)
                  ↓
         ┌─────────────────────────┐
         │  global_planner_node    │
         │  (global_planner)       │
         └─────────────────────────┘
                  ↑        ↓
         /odom    │        │ /planned_path (Path)
    (Odometry)    │        │
                  │        ↓
         ┌────────┴────┐  ┌──────────────┐
         │   Robot     │  │    RViz      │
         │  (odom pub) │  │ (visualiza)  │
         └─────────────┘  └──────┬───────┘
                                 │
                     /goal_pose  │
                    (PoseStamped)│
                                 ↓
                    ┌────────────────────┐
                    │ Usuario (2D Goal)  │
                    └────────────────────┘
```

### Nodos

| Nodo | Paquete | Descripción |
|------|---------|-------------|
| `global_planner_node` | `global_planner` | Nodo principal de planificación con algoritmo de Dijkstra |
| `map_server` | `nav2_map_server` | Publica el mapa estático del entorno (OccupancyGrid) |
| `lifecycle_manager_mapper` | `nav2_lifecycle_manager` | Gestiona el ciclo de vida del map_server |

### Tópicos

| Nombre | Tipo | Publicador | Suscriptor | Descripción |
|--------|------|------------|------------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | `map_server` | `global_planner_node` | Mapa de ocupación del entorno |
| `/odom` | `nav_msgs/Odometry` | Robot/Simulador | `global_planner_node` | Odometría del robot (pose actual) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz | `global_planner_node` | Objetivo 2D definido por usuario |
| `/planned_path` | `nav_msgs/Path` | `global_planner_node` | RViz | Trayectoria calculada |

### Parámetros configurables

| Parámetro | Tipo | Default | Descripción |
|-----------|------|---------|-------------|
| `inflation_radius` | float | 0.3 | Radio de inflación de obstáculos (metros) |
| `map_topic` | string | `/map` | Tópico del mapa |
| `odom_topic` | string | `/odom` | Tópico de odometría |
| `goal_topic` | string | `/goal_pose` | Tópico del objetivo |
| `path_topic` | string | `/planned_path` | Tópico de la trayectoria |

---

## 📦 Dependencias

### Software requerido

| Dependencia | Versión mínima | Descripción |
|-------------|----------------|-------------|
| ROS 2 | Humble/Iron/Jazzy | Framework de robótica |
| Python | 3.8+ | Lenguaje de programación |
| NumPy | 1.20+ | Procesamiento de arrays numéricos |
| Nav2 Map Server | ROS 2 | Servidor de mapas estáticos |
| Nav2 Lifecycle Manager | ROS 2 | Gestor de ciclo de vida de nodos |

### Instalación de dependencias

```bash
# Paquetes ROS 2
sudo apt install ros-${ROS_DISTRO}-nav2-map-server \
                 ros-${ROS_DISTRO}-nav2-lifecycle-manager \
                 ros-${ROS_DISTRO}-nav-msgs \
                 ros-${ROS_DISTRO}-geometry-msgs \
                 ros-${ROS_DISTRO}-sensor-msgs

# Paquetes Python
pip3 install numpy
```

---

## 🚀 Instalación, Compilación y Ejecución

### Paso 1: Clonar el repositorio

```bash
cd ~/ros2_ws/src
git clone <URL_DEL_REPOSITORIO> global_planner
```

### Paso 2: Instalar dependencias

```bash
cd ~/ros2_ws

# Instalar dependencias automáticamente con rosdep
rosdep install --from-paths src --ignore-src -r -y

# Instalar NumPy si no está instalado
pip3 install numpy
```

### Paso 3: Compilar el paquete

```bash
cd ~/ros2_ws
colcon build --packages-select global_planner
source install/setup.bash
```

### Paso 4: Ejecutar el sistema

```bash
# Lanzar el sistema completo (map_server + planificador)
ros2 launch global_planner planner_with_map_launch.py
```

> **Nota**: El mapa ya está incluido en `maps/` del repositorio, no necesitas añadirlo.

### Paso 5: Visualizar en RViz (terminal separada)

```bash
rviz2
```

**Configuración de RViz:**
1. Fixed Frame → `map`
2. Add → By topic → `/map` → Map
3. Add → By topic → `/planned_path` → Path
4. Toolbar → "2D Goal Pose" → Clic en el mapa para definir objetivos

---

## 🎮 Launch Files

### 1. `planner_with_map_launch.py` (Recomendado)

Lanza el sistema completo incluyendo map_server, lifecycle_manager y el nodo de planificación. Carga automáticamente el mapa desde `maps/map.yaml`.

```bash
ros2 launch global_planner planner_with_map_launch.py
```

### 2. `global_planner_launch.py`

Lanza únicamente el nodo de planificación. Requiere que map_server esté corriendo externamente.

```bash
ros2 launch global_planner global_planner_launch.py
```

### Argumentos disponibles

```bash
# Usar tiempo de simulación (para Gazebo)
ros2 launch global_planner planner_with_map_launch.py use_sim_time:=true

# Especificar mapa personalizado
ros2 launch global_planner planner_with_map_launch.py map:=/ruta/a/tu/mapa.yaml

# Especificar archivo de parámetros personalizado
ros2 launch global_planner planner_with_map_launch.py params_file:=/ruta/a/params.yaml
```

---

## 🔍 Verificación del Sistema

### Ver nodos activos

```bash
ros2 node list
```

Deberías ver:
- `/map_server`
- `/global_planner_node`
- `/lifecycle_manager_mapper`

### Ver tópicos activos

```bash
ros2 topic list
```

Deberías ver:
- `/map`
- `/odom`
- `/goal_pose`
- `/planned_path`

### Monitorear la trayectoria publicada

```bash
ros2 topic echo /planned_path
```

### Ver información del nodo de planificación

```bash
ros2 node info /global_planner_node
```

### Enviar objetivo de prueba por comando

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}" --once
```

---

## 🛠️ Solución de Problemas

### El nodo no encuentra trayectorias

**Posibles causas:**
- El mapa no se está publicando correctamente
- El `inflation_radius` es muy grande y bloquea caminos
- El objetivo está dentro de un obstáculo

**Solución:**
```bash
# Verificar que el mapa se publica
ros2 topic echo /map --once

# Verificar parámetros del planificador
ros2 param list /global_planner_node
```

### La trayectoria no se visualiza en RViz

**Solución:**
1. Verificar que el `Fixed Frame` sea `map`
2. Añadir display de tipo `Path` en RViz con topic `/planned_path`
3. Verificar que el nodo está publicando:
   ```bash
   ros2 topic hz /planned_path
   ```

### "No map received"

**Solución:**
- Asegúrate de usar el launch completo: `planner_with_map_launch.py`
- Verifica que el map_server está corriendo:
  ```bash
  ros2 node list | grep map_server
  ```

### El robot no se mueve

**Aclaración**: Este paquete solo realiza **planificación global**, no control del robot. Para que el robot siga la trayectoria necesitas:
- Un controlador de trayectoria (como `nav2_controller` de Nav2)
- O implementar tu propio seguidor de trayectoria que lea `/planned_path`

---

## 📁 Estructura del Proyecto

```
global_planner/
├── config/
│   └── planner_params.yaml          # Parámetros configurables
├── global_planner/
│   ├── __init__.py                  # Módulo Python
│   └── global_planner_node.py       # Nodo principal con Dijkstra
├── launch/
│   ├── global_planner_launch.py     # Launch solo planificador
│   └── planner_with_map_launch.py   # Launch completo (recomendado)
├── maps/
│   ├── map.yaml                     # Configuración del mapa (incluido)
│   └── map.pgm                      # Imagen del mapa (incluido)
├── resource/
│   └── global_planner               # Resource marker
├── .gitignore                       # Control de versiones
├── package.xml                      # Manifest ROS 2
├── setup.py                         # Setup Python
├── setup.cfg                        # Configuración setup
└── README.md                        # Este archivo
```

---

## 📚 Detalles de Implementación

### Clase `DijkstraPlanner`

Implementa el algoritmo de Dijkstra sobre un grid 2D:

**Métodos principales:**
- `__init__(occupancy_grid, inflation_radius)`: Inicializa con el mapa y radio de inflación
- `inflate_obstacles(inflation_radius)`: Expande obstáculos considerando tamaño del robot
- `world_to_grid(x, y)`: Convierte coordenadas del mundo a índices de grid
- `grid_to_world(grid_x, grid_y)`: Convierte índices de grid a coordenadas del mundo
- `is_valid(grid_x, grid_y)`: Verifica si una celda es válida (libre y dentro del mapa)
- `heuristic(a, b)`: Retorna 0.0 (sin heurística para Dijkstra puro)
- `get_neighbors(pos)`: Retorna vecinos válidos con sus costos (8-conectividad)
- `plan(start, goal)`: Ejecuta Dijkstra y retorna la trayectoria óptima

**Características:**
- Sin heurística: Explora uniformemente (garantiza optimalidad)
- Inflación de obstáculos: Radio configurable para seguridad
- 8-conectividad: Movimientos diagonales permitidos
- Cola de prioridad eficiente: `heapq` de Python

### Clase `GlobalPlannerNode`

Nodo ROS 2 que integra el planificador con el sistema:

**Callbacks:**
- `map_callback(msg)`: Recibe el mapa y crea instancia de DijkstraPlanner
- `odom_callback(msg)`: Actualiza la pose actual del robot
- `goal_callback(msg)`: Recibe objetivo, planifica y publica trayectoria

**Características:**
- Gestión de estado: Mantiene pose, mapa y planificador actualizados
- Validación: Verifica que hay mapa y pose antes de planificar
- Publicación: Genera `nav_msgs/Path` con todas las poses de la trayectoria
- Recalculación automática: Cada nuevo objetivo genera nueva trayectoria

---

## 📝 Notas Importantes

1. **Mapa incluido**: El repositorio incluye un mapa de ejemplo en `maps/`. Puedes reemplazarlo con tu propio mapa.

2. **Solo planificación**: Este paquete NO incluye control de movimiento. Solo calcula y publica la trayectoria.

3. **Requiere odometría**: Necesitas un nodo que publique `/odom` (robot real o simulador).

4. **Reproducibilidad**: Siguiendo los pasos de este README, el sistema debe funcionar sin modificaciones adicionales.

---

## 👤 Autor

- **Nombre**: Dereck
- **Proyecto**: Parte B - Planificación Global de Trayectoria
- **Curso**: Robótica Móvil - 2do Parcial
- **Fecha**: Enero 2026

---

## 📄 Licencia

MIT License
