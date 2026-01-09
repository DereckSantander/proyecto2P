#!/usr/bin/env python3
"""
Launch file para el sistema de planificación global
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Genera la descripción del launch
    """
    # Obtener ruta del paquete
    pkg_dir = get_package_share_directory('global_planner')
    
    # Ruta al archivo de parámetros
    params_file = os.path.join(pkg_dir, 'config', 'planner_params.yaml')
    
    # Declarar argumentos
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo de simulación'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Ruta al archivo de parámetros'
    )
    
    # Nodo de planificación global
    global_planner_node = Node(
        package='global_planner',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        global_planner_node
    ])
