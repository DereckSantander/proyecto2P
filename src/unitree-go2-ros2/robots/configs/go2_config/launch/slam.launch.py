import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    go2_config_pkg = FindPackageShare('go2_config')
    slam_toolbox_pkg = FindPackageShare('slam_toolbox')
    champ_nav_pkg = FindPackageShare('champ_navigation')

    slam_params_file = PathJoinSubstitution(
        [go2_config_pkg, 'config/autonomy', 'slam.yaml']
    )

    slam_launch_path = PathJoinSubstitution(
        [slam_toolbox_pkg, 'launch', 'online_async_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [champ_nav_pkg, 'rviz', 'slam.rviz']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_toggle = LaunchConfiguration('rviz')

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true'
    )

    # 🔥 SOLO SLAM (nada de Nav2 aquí)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
            'mode': 'mapping'   # 🔴 CLAVE
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(rviz_toggle),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_sim_time,
        declare_rviz,
        slam_launch,
        rviz_node
    ])

