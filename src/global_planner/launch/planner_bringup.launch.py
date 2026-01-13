"""
Launch file principal para el sistema de planificación global
Lanza:
  - Map Server (carga el mapa)
  - Robot State Publisher (visualización del robot)
  - Dijkstra Planner (planificación de rutas)
  - RViz (visualización)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paquetes
    global_planner_dir = get_package_share_directory('global_planner')
    
    # Argumentos
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(global_planner_dir, 'maps', 'map.yaml'),
        description='Ruta completa al archivo map.yaml'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo de simulación'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(global_planner_dir, 'config', 'planner.rviz'),
        description='Ruta al archivo de configuración de RViz'
    )
    
    # Nodo Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Lifecycle Manager para el Map Server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # Nodo Dijkstra Planner
    dijkstra_planner_node = Node(
        package='global_planner',
        executable='dijkstra_planner',
        name='dijkstra_planner',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_diagonal': True,
            'occupied_threshold': 65,
            'inflation_radius': 0.3,
            'path_resolution': 0.05
        }]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        rviz_config_arg,
        map_server_node,
        lifecycle_manager_node,
        dijkstra_planner_node,
        rviz_node
    ])
