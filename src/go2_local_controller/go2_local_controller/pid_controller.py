"""
Local PID Controller for following a global Path in ROS 2.
Subscriptions: /odom (nav_msgs/Odometry), /global_path (nav_msgs/Path)
Publishes: /cmd_vel (geometry_msgs/Twist)
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Parameters (use_sim_time is automatically handled by ROS 2)
        self.declare_parameter('lookahead_distance', 1.5)  # Más lookahead para curvas suaves
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('yaw_tolerance', 0.3)

        self.declare_parameter('lin_kp', 0.6)  # Más suave
        self.declare_parameter('lin_ki', 0.01)  
        self.declare_parameter('lin_kd', 0.2)  

        self.declare_parameter('ang_kp', 1.0)  # Reducido aún más
        self.declare_parameter('ang_ki', 0.005)
        self.declare_parameter('ang_kd', 0.15)  

        self.declare_parameter('max_linear_speed', 0.4)  # Más lento para seguridad
        self.declare_parameter('max_angular_speed', 0.8)  # Giros mucho más lentos
        
        self.declare_parameter('integral_limit', 0.3)  # Anti-windup más estricto
        self.declare_parameter('velocity_smoothing', 0.5)  # Más suavizado
        self.declare_parameter('min_linear_speed', 0.05)  # Velocidad mínima para avanzar

        self.declare_parameter('control_rate_hz', 20.0)

        # Read parameters
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)

        self.lin_kp = float(self.get_parameter('lin_kp').value)
        self.lin_ki = float(self.get_parameter('lin_ki').value)
        self.lin_kd = float(self.get_parameter('lin_kd').value)

        self.ang_kp = float(self.get_parameter('ang_kp').value)
        self.ang_ki = float(self.get_parameter('ang_ki').value)
        self.ang_kd = float(self.get_parameter('ang_kd').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        
        self.integral_limit = float(self.get_parameter('integral_limit').value)
        self.velocity_smoothing = float(self.get_parameter('velocity_smoothing').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        # State
        self._odom: Optional[Odometry] = None
        self._path: Optional[Path] = None
        self._prev_lin_err = 0.0
        self._prev_ang_err = 0.0
        self._lin_err_integral = 0.0
        self._ang_err_integral = 0.0
        self._last_time = self.get_clock().now()
        
        # Metrics
        self._total_distance = 0.0  # Distancia total recorrida en metros
        self._prev_x = None
        self._prev_y = None
        self._trajectory_start_time = None  # Tiempo de inicio de la trayectoria
        self._trajectory_active = False
        
        # Smoothing
        self._prev_v = 0.0
        self._prev_w = 0.0

        # QoS
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.create_subscription(Path, '/global_path', self._path_cb, qos)

        # Publisher
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)

        # Control loop
        self._timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)

        self.get_logger().info('✅ PID Controller iniciado')

    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    def _path_cb(self, msg: Path):
        # Al recibir un nuevo path, reiniciar métricas
        # Detectamos nuevo path si: no hay path previo, o el nuevo tiene diferente número de poses, o ya se completó la anterior
        is_new_path = (
            self._path is None or 
            len(self._path.poses) == 0 or 
            len(msg.poses) != len(self._path.poses) or
            not self._trajectory_active
        )
        
        if len(msg.poses) > 0 and is_new_path:
            self.get_logger().info('🎯 Nueva trayectoria recibida - Iniciando métricas')
            self._total_distance = 0.0
            self._prev_x = None
            self._prev_y = None
            self._trajectory_start_time = self.get_clock().now()
            self._trajectory_active = True
        self._path = msg

    def _control_loop(self):
        if self._odom is None or self._path is None or len(self._path.poses) == 0:
            return

        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1.0 / self.control_rate_hz
        self._last_time = now

        # Current pose
        cx = self._odom.pose.pose.position.x
        cy = self._odom.pose.pose.position.y
        cyaw = self._yaw_from_quat(self._odom.pose.pose.orientation)
        
        # Actualizar distancia recorrida
        if self._trajectory_active:
            if self._prev_x is not None and self._prev_y is not None:
                segment_dist = math.hypot(cx - self._prev_x, cy - self._prev_y)
                self._total_distance += segment_dist
            self._prev_x = cx
            self._prev_y = cy

        # Target waypoint from path
        target = self._find_target_waypoint(cx, cy)
        if target is None:
            # Fallback: use last waypoint
            last = self._path.poses[-1].pose.position
            target = (last.x, last.y)

        tx, ty = target

        # Position and heading errors
        dx = tx - cx
        dy = ty - cy
        dist_err = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_err = self._normalize_angle(desired_yaw - cyaw)

        # Goal reached -> stop
        if dist_err < self.goal_tolerance and abs(yaw_err) < self.yaw_tolerance:
            if self._trajectory_active:
                # Calcular tiempo total
                elapsed_time = (self.get_clock().now() - self._trajectory_start_time).nanoseconds / 1e9
                self.get_logger().info('='*60)
                self.get_logger().info('🏁 OBJETIVO ALCANZADO')
                self.get_logger().info(f'📏 Distancia total recorrida: {self._total_distance:.2f} metros')
                self.get_logger().info(f'⏱️  Tiempo total: {elapsed_time:.2f} segundos')
                self.get_logger().info(f'🚀 Velocidad promedio: {self._total_distance/elapsed_time:.2f} m/s')
                self.get_logger().info('='*60)
                self._trajectory_active = False
            self._publish_cmd(0.0, 0.0)
            return

        # Si el error de orientación es grande, priorizar giro sobre avance
        if abs(yaw_err) > 0.5:  # ~30 grados
            # Casi detener avance para alinearse primero
            v = self.min_linear_speed
            w = self.ang_kp * yaw_err * 0.7  # Giro controlado
        else:
            # PID linear con anti-windup
            self._lin_err_integral += dist_err * dt
            self._lin_err_integral = max(-self.integral_limit, min(self.integral_limit, self._lin_err_integral))
            lin_derivative = (dist_err - self._prev_lin_err) / dt
            v = self.lin_kp * dist_err + self.lin_ki * self._lin_err_integral + self.lin_kd * lin_derivative

            # PID angular con anti-windup
            self._ang_err_integral += yaw_err * dt
            self._ang_err_integral = max(-self.integral_limit, min(self.integral_limit, self._ang_err_integral))
            ang_derivative = (yaw_err - self._prev_ang_err) / dt
            w = self.ang_kp * yaw_err + self.ang_ki * self._ang_err_integral + self.ang_kd * ang_derivative

            # Desaceleración progresiva al acercarse al objetivo
            if dist_err < 1.5:
                speed_scale = max(0.3, dist_err / 1.5)  # No bajar de 30%
                v *= speed_scale

            # Acoplamiento fuerte: reducir mucho la velocidad al girar
            angular_ratio = abs(yaw_err) / 0.5  # Normalizar a 0.5 rad
            turn_reduction = max(0.3, 1.0 - angular_ratio * 0.7)  # Reducir hasta 70%
            v *= turn_reduction

        # Límites de velocidad
        w = max(-self.max_angular_speed, min(self.max_angular_speed, w))
        v = max(self.min_linear_speed, min(self.max_linear_speed, v))
        
        # Suavizado de comandos (filtro de primer orden)
        alpha = self.velocity_smoothing
        v = alpha * v + (1.0 - alpha) * self._prev_v
        w = alpha * w + (1.0 - alpha) * self._prev_w
        
        # Guardar para siguiente iteración
        self._prev_v = v
        self._prev_w = w

        # Update previous errors
        self._prev_lin_err = dist_err
        self._prev_ang_err = yaw_err

        self._publish_cmd(v, w)

    def _publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self._cmd_pub.publish(msg)

    @staticmethod
    def _yaw_from_quat(q) -> float:
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _find_target_waypoint(self, cx: float, cy: float) -> Optional[Tuple[float, float]]:
        # choose closest waypoint then lookahead by distance
        closest_idx = 0
        closest_dist = float('inf')
        for i, ps in enumerate(self._path.poses):
            px = ps.pose.position.x
            py = ps.pose.position.y
            d = math.hypot(px - cx, py - cy)
            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        # advance until lookahead distance reached
        accum = 0.0
        lastx = self._path.poses[closest_idx].pose.position.x
        lasty = self._path.poses[closest_idx].pose.position.y
        for j in range(closest_idx + 1, len(self._path.poses)):
            px = self._path.poses[j].pose.position.x
            py = self._path.poses[j].pose.position.y
            accum += math.hypot(px - lastx, py - lasty)
            lastx, lasty = px, py
            if accum >= self.lookahead_distance:
                return (px, py)

        # if not reached, return last point
        last = self._path.poses[-1].pose.position
        return (last.x, last.y)


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
