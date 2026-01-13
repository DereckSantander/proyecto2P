"""
Launch file solo para el Map Server
Útil para lanzar el mapa independientemente
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    global_planner_dir = get_package_share_directory('global_planner')
    
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
    
    # Lifecycle Manager
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
    
    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        map_server_node,
        lifecycle_manager_node
    ])
