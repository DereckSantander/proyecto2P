"""
Launch file simplificado - solo lanza el planificador y RViz
Asume que el mapa y el robot ya están corriendo
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    global_planner_dir = get_package_share_directory('global_planner')
    
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
        use_sim_time_arg,
        rviz_config_arg,
        dijkstra_planner_node,
        rviz_node
    ])
