#!/usr/bin/env python3
"""
Nodo de planificación global de trayectoria usando Dijkstra

Este nodo implementa:
- Suscripción a /odom (pose del robot)
- Suscripción a /goal_pose (objetivo 2D desde RViz)
- Suscripción a /map (mapa del entorno)
- Algoritmo de Dijkstra para planificación global
- Publicación de nav_msgs/Path
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import heapq
from typing import List, Tuple, Optional
import math


class DijkstraPlanner:
    """
    Implementación del algoritmo de Dijkstra para planificación en grid 2D
    """
    
    def __init__(self, occupancy_grid: OccupancyGrid, inflation_radius: float = 0.3):
        """
        Inicializa el planificador de Dijkstra
        
        Args:
            occupancy_grid: Mapa de ocupación
            inflation_radius: Radio de inflación de obstáculos en metros
        """
        self.map = occupancy_grid
        self.resolution = occupancy_grid.info.resolution
        self.width = occupancy_grid.info.width
        self.height = occupancy_grid.info.height
        self.origin = occupancy_grid.info.origin
        
        # Convertir datos del mapa a matriz 2D
        self.grid = np.array(occupancy_grid.data).reshape((self.height, self.width))
        
        # Inflar obstáculos
        self.inflate_obstacles(inflation_radius)
        
    def inflate_obstacles(self, inflation_radius: float):
        """
        Infla los obstáculos en el mapa para tener en cuenta el tamaño del robot
        
        Args:
            inflation_radius: Radio de inflación en metros
        """
        inflation_cells = int(inflation_radius / self.resolution)
        inflated_grid = self.grid.copy()
        
        for i in range(self.height):
            for j in range(self.width):
                if self.grid[i, j] > 50:  # Obstáculo detectado
                    # Inflar alrededor del obstáculo
                    for di in range(-inflation_cells, inflation_cells + 1):
                        for dj in range(-inflation_cells, inflation_cells + 1):
                            ni, nj = i + di, j + dj
                            if 0 <= ni < self.height and 0 <= nj < self.width:
                                if di*di + dj*dj <= inflation_cells*inflation_cells:
                                    inflated_grid[ni, nj] = 100
        
        self.grid = inflated_grid
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convierte coordenadas del mundo a coordenadas de grid
        
        Args:
            x, y: Coordenadas en el mundo
            
        Returns:
            Tupla (grid_x, grid_y)
        """
        grid_x = int((x - self.origin.position.x) / self.resolution)
        grid_y = int((y - self.origin.position.y) / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Convierte coordenadas de grid a coordenadas del mundo
        
        Args:
            grid_x, grid_y: Coordenadas en el grid
            
        Returns:
            Tupla (x, y) en coordenadas del mundo
        """
        x = grid_x * self.resolution + self.origin.position.x
        y = grid_y * self.resolution + self.origin.position.y
        return x, y
    
    def is_valid(self, grid_x: int, grid_y: int) -> bool:
        """
        Verifica si una celda es válida (dentro del mapa y libre)
        
        Args:
            grid_x, grid_y: Coordenadas de la celda
            
        Returns:
            True si la celda es válida
        """
        if grid_x < 0 or grid_x >= self.width or grid_y < 0 or grid_y >= self.height:
            return False
        return self.grid[grid_y, grid_x] < 50  # Celda libre si valor < 50
    
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """
        Heurística para Dijkstra (siempre 0, lo que lo hace Dijkstra puro)
        
        Args:
            a, b: Puntos (no utilizados en Dijkstra)
            
        Returns:
            0.0 (sin heurística)
        """
        return 0.0  # Dijkstra no usa heurística
    
    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int, float]]:
        """
        Obtiene los vecinos válidos de una posición
        
        Args:
            pos: Posición actual (grid_x, grid_y)
            
        Returns:
            Lista de tuplas (vecino_x, vecino_y, costo)
        """
        neighbors = []
        x, y = pos
        
        # 8 direcciones: arriba, abajo, izquierda, derecha y diagonales
        directions = [
            (1, 0, 1.0),    # derecha
            (-1, 0, 1.0),   # izquierda
            (0, 1, 1.0),    # arriba
            (0, -1, 1.0),   # abajo
            (1, 1, 1.414),  # diagonal derecha-arriba
            (1, -1, 1.414), # diagonal derecha-abajo
            (-1, 1, 1.414), # diagonal izquierda-arriba
            (-1, -1, 1.414) # diagonal izquierda-abajo
        ]
        
        for dx, dy, cost in directions:
            nx, ny = x + dx, y + dy
            if self.is_valid(nx, ny):
                neighbors.append((nx, ny, cost))
        
        return neighbors
    
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Planifica una trayectoria desde start hasta goal usando Dijkstra
        
        Args:
            start: Posición inicial (x, y) en coordenadas del mundo
            goal: Posición objetivo (x, y) en coordenadas del mundo
            
        Returns:
            Lista de puntos (x, y) que forman la trayectoria, o None si no hay solución
        """
        # Convertir a coordenadas de grid
        start_grid = self.world_to_grid(start[0], start[1])
        goal_grid = self.world_to_grid(goal[0], goal[1])
        
        # Verificar que inicio y objetivo sean válidos
        if not self.is_valid(start_grid[0], start_grid[1]):
            return None
        if not self.is_valid(goal_grid[0], goal_grid[1]):
            return None
        
        # Inicializar Dijkstra
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}
        # En Dijkstra, f_score = g_score (sin heurística)
        f_score = {start_grid: 0}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            # Si llegamos al objetivo
            if current == goal_grid:
                # Reconstruir trayectoria
                path = []
                while current in came_from:
                    world_pos = self.grid_to_world(current[0], current[1])
                    path.append(world_pos)
                    current = came_from[current]
                # Añadir el punto inicial
                world_pos = self.grid_to_world(start_grid[0], start_grid[1])
                path.append(world_pos)
                path.reverse()
                return path
            
            # Explorar vecinos
            for neighbor_x, neighbor_y, move_cost in self.get_neighbors(current):
                neighbor = (neighbor_x, neighbor_y)
                tentative_g_score = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    # Dijkstra usa solo el costo acumulado, sin heurística
                    f_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No se encontró trayectoria
        return None


class GlobalPlannerNode(Node):
    """
    Nodo ROS 2 para planificación global de trayectoria
    """
    
    def __init__(self):
        super().__init__('global_planner_node')
        
        # Declarar parámetros
        self.declare_parameter('inflation_radius', 0.3)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')
        
        # Obtener parámetros
        self.inflation_radius = self.get_parameter('inflation_radius').value
        map_topic = self.get_parameter('map_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        path_topic = self.get_parameter('path_topic').value
        
        # Variables de estado
        self.current_pose: [OptiDijkstraPose] = None
        self.map: Optional[OccupancyGrid] = None
        self.planner: Optional[AStarPlanner] = None
        
        # Suscriptores
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            goal_topic,
            self.goal_callback,
            10
        )
        
        # Publicador de trayectoria
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        
        self.get_logger().info('Nodo de planificación global iniciado')
        self.get_logger().info(f'  - Esperando mapa en: {map_topic}')
        self.get_logger().info(f'  - Esperando odometría en: {odom_topic}')
        self.get_logger().info(f'  - Esperando objetivo en: {goal_topic}')
        self.get_logger().info(f'  - Publicando trayectoria en: {path_topic}')
    
    def map_callback(self, msg: OccupancyGrid):
        """
        Callback para el mapa
        
        Args:
            msg: Mensaje del mapa
        """
        self.map = msgDijkstra
        self.planner = AStarPlanner(msg, self.inflation_radius)
        self.get_logger().info(f'Mapa recibido: {msg.info.width}x{msg.info.height} celdas, resolución {msg.info.resolution}m')
    
    def odom_callback(self, msg: Odometry):
        """
        Callback para la odometría
        
        Args:
            msg: Mensaje de odometría
        """
        self.current_pose = msg.pose.pose
    
    def goal_callback(self, msg: PoseStamped):
        """
        Callback para el objetivo
        
        Args:
            msg: Mensaje con el objetivo
        """
        self.get_logger().info(f'Nuevo objetivo recibido: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        
        # Verificar que tenemos todo lo necesario
        if self.current_pose is None:
            self.get_logger().warn('No se ha recibido la pose actual del robot')
            return
        
        if self.planner is None:
            self.get_logger().warn('No se ha recibido el mapa')
            return
        
        # Planificar trayectoria
        start = (self.current_pose.position.x, self.current_pose.position.y)
        goal = (msg.pose.position.x, msg.pose.position.y)
        
        self.get_logger().info(f'Planificando trayectoria desde ({start[0]:.2f}, {start[1]:.2f}) hasta ({goal[0]:.2f}, {goal[1]:.2f})')
        
        path_points = self.planner.plan(start, goal)
        
        if path_points is None:
            self.get_logger().error('No se encontró una trayectoria válida')
            return
        
        # Crear mensaje Path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for x, y in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        # Publicar trayectoria
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Trayectoria publicada con {len(path_points)} puntos')


def main(args=None):
    """
    Función principal
    """
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
