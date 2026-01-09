#!/usr/bin/env python3
"""
Launch file completo con map_server y planificador global
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Genera la descripción del launch con map_server y planificador
    """
    # Obtener ruta del paquete
    pkg_dir = get_package_share_directory('global_planner')
    
    # Rutas a archivos
    params_file = os.path.join(pkg_dir, 'config', 'planner_params.yaml')
    map_file = os.path.join(pkg_dir, '..', '..', '..', '..', 'src', 'global_planner', 'maps', 'map.yaml')
    
    # Declarar argumentos
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo de simulación'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Ruta completa al archivo map.yaml'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Ruta al archivo de parámetros'
    )
    
    # Nodo map_server para publicar el mapa
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        emulate_tty=True
    )
    
    # Nodo lifecycle manager para activar map_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server']
        }],
        emulate_tty=True
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
        map_file_arg,
        params_file_arg,
        map_server_node,
        lifecycle_manager_node,
        global_planner_node
    ])
