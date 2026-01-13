"""
Global Path Planner Node using Dijkstra Algorithm
Subscribes to: /odom, /goal_pose, /map
Publishes: /global_path
Saves waypoints to CSV
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Point
import numpy as np
import heapq
import math
import csv
import os
from datetime import datetime
from pathlib import Path as PathlibPath


class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__('dijkstra_planner')
        
        self.get_logger().info('='*60)
        self.get_logger().info('🚀 INICIALIZANDO PLANIFICADOR DIJKSTRA')
        self.get_logger().info('='*60)
        
        # Parámetros del planificador
        self.declare_parameter('use_diagonal', True)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('inflation_radius', 0.3)  # metros
        self.declare_parameter('path_resolution', 0.05)  # metros
        
        self.use_diagonal = self.get_parameter('use_diagonal').value
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.path_resolution = self.get_parameter('path_resolution').value
        
        self.get_logger().info('✅ Parámetros cargados correctamente')
        self.get_logger().info(f'   - Diagonal: {self.use_diagonal}')
        self.get_logger().info(f'   - Umbral ocupación: {self.occupied_threshold}')
        self.get_logger().info(f'   - Radio inflación: {self.inflation_radius}m')
        self.get_logger().info(f'   - Resolución path: {self.path_resolution}m')
        
        # Estado del sistema
        self.map_data = None
        self.map_info = None
        self.current_pose = None
        self.goal_pose = None
        self.map_array = None
        self.inflated_map = None
        
        # Contadores
        self.map_received = False
        self.odom_received = False
        self.goal_received = False
        
        # Contador de trayectorias guardadas
        self.trajectory_count = 0
        
        # Crear directorio para CSV si no existe
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.csv_dir = os.path.join(os.path.expanduser('~/proyecto2p/src/global_planner'), 'waypoints')
        
        try:
            os.makedirs(self.csv_dir, exist_ok=True)
            self.get_logger().info(f'✅ Directorio CSV creado: {self.csv_dir}')
        except Exception as e:
            self.get_logger().error(f'❌ Error al crear directorio CSV: {e}')
            self.get_logger().warn(f'Usando directorio temporal: /tmp/waypoints')
            self.csv_dir = '/tmp/waypoints'
            os.makedirs(self.csv_dir, exist_ok=True)
        
        # QoS Profile para tópicos
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS especial para /map (necesita recibir mensajes anteriores)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.get_logger().info('📡 Subscribiendo a tópicos...')
        
        # Subscripciones
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        self.get_logger().info('   ✓ Suscrito a /odom')
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            qos_profile
        )
        self.get_logger().info('   ✓ Suscrito a /goal_pose')
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        self.get_logger().info('   ✓ Suscrito a /map (TRANSIENT_LOCAL)')
        
        # Publicador del path
        self.path_pub = self.create_publisher(
            Path,
            '/global_path',
            qos_profile
        )
        self.get_logger().info('   ✓ Publicador /global_path creado')
        
        # Timer para verificar estado
        self.status_timer = self.create_timer(5.0, self.check_status)
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ Algoritmo implementado')
        self.get_logger().info('='*60)
        self.get_logger().info(f'📁 Waypoints se guardarán en: {self.csv_dir}')
    
    def check_status(self):
        """Verificar estado del sistema periódicamente"""
        status = []
        
        if self.map_received:
            status.append('✅ Mapa')
        else:
            status.append('❌ Mapa')
        
        if self.odom_received:
            status.append('✅ Odometría')
        else:
            status.append('❌ Odometría')
        
        if self.goal_received:
            status.append('✅ Objetivo')
        else:
            status.append('❌ Objetivo')
        
        self.get_logger().info(f'📊 Estado: {" | ".join(status)}')
    
    def save_waypoints_to_csv(self, path, filename=None):
        """Guardar waypoints en archivo CSV"""
        if filename is None:
            # Crear nombre con timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"waypoints_{timestamp}.csv"
        
        filepath = os.path.join(self.csv_dir, filename)
        
        self.get_logger().info(f'🔍 Intentando guardar CSV en: {filepath}')
        self.get_logger().info(f'   Directorio existe: {os.path.exists(self.csv_dir)}')
        self.get_logger().info(f'   Directorio escribible: {os.access(self.csv_dir, os.W_OK)}')
        
        try:
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Escribir encabezado
                writer.writerow(['waypoint_id', 'x', 'y', 'z'])
                
                # Escribir cada waypoint
                for idx, pose in enumerate(path.poses):
                    writer.writerow([
                        idx,
                        f"{pose.pose.position.x:.4f}",
                        f"{pose.pose.position.y:.4f}",
                        f"{pose.pose.position.z:.4f}"
                    ])
            
            self.trajectory_count += 1
            self.get_logger().info('='*60)
            self.get_logger().info(f'✅ 💾 WAYPOINTS GUARDADOS EXITOSAMENTE!')
            self.get_logger().info(f'   Archivo: {filepath}')
            self.get_logger().info(f'   Total de waypoints: {len(path.poses)}')
            self.get_logger().info(f'   Total de trayectorias guardadas: {self.trajectory_count}')
            self.get_logger().info('='*60)
        except Exception as e:
            self.get_logger().error('='*60)
            self.get_logger().error(f'❌ ERROR AL GUARDAR CSV')
            self.get_logger().error(f'   Excepción: {type(e).__name__}: {e}')
            self.get_logger().error(f'   Ruta intenta: {filepath}')
            self.get_logger().error('='*60)
    
    def odom_callback(self, msg):
        """Callback para actualizar la pose actual del robot"""
        if not self.odom_received:
            self.get_logger().info('✅ Odometría recibida correctamente')
            self.odom_received = True
        self.current_pose = msg.pose.pose
    
    def goal_callback(self, msg):
        """Callback cuando se recibe un nuevo objetivo desde RViz"""
        if not self.goal_received:
            self.get_logger().info('✅ Objetivo recibido correctamente')
            self.goal_received = True
        
        self.get_logger().info(f'🎯 Nuevo objetivo: x={msg.pose.position.x:.2f}, '
                             f'y={msg.pose.position.y:.2f}')
        self.goal_pose = msg.pose
        
        # Intentar planificar inmediatamente
        if self.current_pose and self.map_data:
            self.plan_path()
        else:
            if not self.current_pose:
                self.get_logger().warn('⚠️  Esperando pose actual del robot...')
            if not self.map_data:
                self.get_logger().warn('⚠️  Esperando mapa de ocupación...')
    
    def map_callback(self, msg):
        """Callback para recibir y procesar el mapa"""
        if not self.map_received:
            self.get_logger().info('='*60)
            self.get_logger().info('✅ Mapa cargado correctamente')
            self.get_logger().info('='*60)
            self.map_received = True
        
        self.map_data = msg
        self.map_info = msg.info
        
        # Convertir datos del mapa a array 2D
        width = msg.info.width
        height = msg.info.height
        self.map_array = np.array(msg.data).reshape((height, width))
        
        # Inflar obstáculos para seguridad
        self.inflated_map = self.inflate_obstacles(self.map_array)
        
        self.get_logger().info(f'📊 Datos del mapa:')
        self.get_logger().info(f'   - Dimensiones: {width}x{height} píxeles')
        self.get_logger().info(f'   - Resolución: {msg.info.resolution}m/píxel')
        self.get_logger().info(f'   - Origen: ({msg.info.origin.position.x:.2f}, '
                             f'{msg.info.origin.position.y:.2f})')
        self.get_logger().info(f'   - Inflación: {self.inflation_radius}m')
        self.get_logger().info(f'   - Obstáculos encontrados: '
                             f'{np.sum(self.map_array >= self.occupied_threshold)}')
    
    def inflate_obstacles(self, map_array):
        """Inflar obstáculos en el mapa para seguridad del robot"""
        inflated = map_array.copy()
        height, width = map_array.shape
        
        # Calcular radio de inflación en celdas
        if self.map_info:
            inflation_cells = int(self.inflation_radius / self.map_info.resolution)
        else:
            inflation_cells = 3
        
        # Encontrar todos los obstáculos
        obstacles = np.argwhere(map_array >= self.occupied_threshold)
        
        # Inflar cada obstáculo
        for obs_y, obs_x in obstacles:
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    # Verificar distancia euclidiana
                    if dx*dx + dy*dy <= inflation_cells*inflation_cells:
                        ny, nx = obs_y + dy, obs_x + dx
                        if 0 <= ny < height and 0 <= nx < width:
                            inflated[ny, nx] = 100
        
        return inflated
    
    def world_to_map(self, x, y):
        """Convertir coordenadas del mundo a coordenadas del mapa"""
        if not self.map_info:
            return None, None
        
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        
        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)
        
        return map_x, map_y
    
    def map_to_world(self, map_x, map_y):
        """Convertir coordenadas del mapa a coordenadas del mundo"""
        if not self.map_info:
            return None, None
        
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        
        world_x = origin_x + (map_x + 0.5) * resolution
        world_y = origin_y + (map_y + 0.5) * resolution
        
        return world_x, world_y
    
    def is_valid_cell(self, x, y):
        """Verificar si una celda es válida y transitable"""
        if self.inflated_map is None:
            return False
            
        height, width = self.inflated_map.shape
        
        # Verificar límites
        if x < 0 or x >= width or y < 0 or y >= height:
            return False
        
        # Verificar si está ocupada
        if self.inflated_map[y, x] >= self.occupied_threshold:
            return False
        
        # Verificar si es desconocida (-1)
        if self.inflated_map[y, x] < 0:
            return False
        
        return True
    
    def get_neighbors(self, x, y):
        """Obtener vecinos válidos de una celda"""
        neighbors = []
        
        # Movimientos: arriba, abajo, izquierda, derecha
        moves = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        
        # Agregar movimientos diagonales si está habilitado
        if self.use_diagonal:
            moves.extend([(1, 1), (1, -1), (-1, 1), (-1, -1)])
        
        for dx, dy in moves:
            nx, ny = x + dx, y + dy
            
            if self.is_valid_cell(nx, ny):
                # Costo: distancia euclidiana
                cost = math.sqrt(dx*dx + dy*dy)
                neighbors.append((nx, ny, cost))
        
        return neighbors
    
    def dijkstra(self, start_x, start_y, goal_x, goal_y):
        """Algoritmo de Dijkstra para encontrar el camino más corto"""
        self.get_logger().info(f'🔍 Ejecutando Dijkstra...')
        self.get_logger().info(f'   - Inicio: ({start_x}, {start_y})')
        self.get_logger().info(f'   - Objetivo: ({goal_x}, {goal_y})')
        
        # Verificar que inicio y objetivo sean válidos
        if not self.is_valid_cell(start_x, start_y):
            self.get_logger().error('❌ Posición inicial no es válida')
            return None
        
        if not self.is_valid_cell(goal_x, goal_y):
            self.get_logger().error('❌ Posición objetivo no es válida')
            return None
        
        # Inicializar estructuras de datos
        open_set = []
        heapq.heappush(open_set, (0, start_x, start_y))
        
        came_from = {}
        cost_so_far = {(start_x, start_y): 0}
        
        visited_count = 0
        
        while open_set:
            current_cost, current_x, current_y = heapq.heappop(open_set)
            visited_count += 1
            
            # Llegamos al objetivo
            if current_x == goal_x and current_y == goal_y:
                self.get_logger().info('='*60)
                self.get_logger().info(f'✅ CAMINO ENCONTRADO!')
                self.get_logger().info(f'   - Nodos visitados: {visited_count}')
                self.get_logger().info('='*60)
                return self.reconstruct_path(came_from, start_x, start_y, goal_x, goal_y)
            
            # Explorar vecinos
            for next_x, next_y, move_cost in self.get_neighbors(current_x, current_y):
                new_cost = cost_so_far[(current_x, current_y)] + move_cost
                
                if (next_x, next_y) not in cost_so_far or new_cost < cost_so_far[(next_x, next_y)]:
                    cost_so_far[(next_x, next_y)] = new_cost
                    heapq.heappush(open_set, (new_cost, next_x, next_y))
                    came_from[(next_x, next_y)] = (current_x, current_y)
        
        self.get_logger().error('='*60)
        self.get_logger().error(f'❌ NO SE ENCONTRÓ CAMINO')
        self.get_logger().error(f'   - Nodos visitados: {visited_count}')
        self.get_logger().error('='*60)
        return None
    
    def reconstruct_path(self, came_from, start_x, start_y, goal_x, goal_y):
        """Reconstruir el camino desde el objetivo hasta el inicio"""
        path = []
        current = (goal_x, goal_y)
        
        while current != (start_x, start_y):
            path.append(current)
            current = came_from[current]
        
        path.append((start_x, start_y))
        path.reverse()
        
        return path
    
    def smooth_path(self, path):
        """Suavizar el camino para hacerlo más natural"""
        if len(path) < 3:
            return path
        
        smoothed = [path[0]]
        
        for i in range(1, len(path) - 1):
            # Simplificar: omitir puntos intermedios en líneas rectas
            prev = path[i - 1]
            curr = path[i]
            next_p = path[i + 1]
            
            # Verificar si los tres puntos están en línea
            dx1 = curr[0] - prev[0]
            dy1 = curr[1] - prev[1]
            dx2 = next_p[0] - curr[0]
            dy2 = next_p[1] - curr[1]
            
            # Si no están alineados, agregar el punto
            if dx1 * dy2 != dy1 * dx2:
                smoothed.append(curr)
        
        smoothed.append(path[-1])
        
        return smoothed
    
    def plan_path(self):
        """Planificar el camino desde la posición actual hasta el objetivo"""
        if not self.current_pose or not self.goal_pose or not self.map_data:
            self.get_logger().warn('⚠️  Faltan datos para planificar')
            return
        
        # Convertir posiciones a coordenadas del mapa
        start_x, start_y = self.world_to_map(
            self.current_pose.position.x,
            self.current_pose.position.y
        )
        
        goal_x, goal_y = self.world_to_map(
            self.goal_pose.position.x,
            self.goal_pose.position.y
        )
        
        if start_x is None or goal_x is None:
            self.get_logger().error('❌ Error al convertir coordenadas')
            return
        
        # Ejecutar Dijkstra
        path = self.dijkstra(start_x, start_y, goal_x, goal_y)
        
        if path is None:
            self.get_logger().error('❌ No se pudo encontrar un camino válido')
            return
        
        # Suavizar el camino
        smoothed_path = self.smooth_path(path)
        
        # Publicar el camino
        self.publish_path(smoothed_path)
    
    def publish_path(self, path):
        """Publicar el camino como nav_msgs/Path y guardar en CSV"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for map_x, map_y in path:
            # Convertir a coordenadas del mundo
            world_x, world_y = self.map_to_world(map_x, map_y)
            
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = world_x
            pose_stamped.pose.position.y = world_y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
        
        # Guardar waypoints en CSV
        self.save_waypoints_to_csv(path_msg)
        
        # Calcular longitud del camino
        length = sum(
            math.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
            for i in range(len(path) - 1)
        ) * self.map_info.resolution
        
        self.get_logger().info('='*60)
        self.get_logger().info('📤 Path publicado en /global_path')
        self.get_logger().info(f'   - Puntos: {len(path_msg.poses)}')
        self.get_logger().info(f'   - Longitud: {length:.2f}m')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    
    planner = DijkstraPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
