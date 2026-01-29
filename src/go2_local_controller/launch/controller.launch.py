"""
Launch: local PID controller for following /global_path.
Assumes /odom and /global_path are already being published.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    lookahead_arg = DeclareLaunchArgument('lookahead_distance', default_value='0.6')
    goal_tol_arg = DeclareLaunchArgument('goal_tolerance', default_value='0.25')
    yaw_tol_arg = DeclareLaunchArgument('yaw_tolerance', default_value='0.25')

    lin_kp_arg = DeclareLaunchArgument('lin_kp', default_value='1.2')
    lin_ki_arg = DeclareLaunchArgument('lin_ki', default_value='0.0')
    lin_kd_arg = DeclareLaunchArgument('lin_kd', default_value='0.0')

    ang_kp_arg = DeclareLaunchArgument('ang_kp', default_value='2.0')
    ang_ki_arg = DeclareLaunchArgument('ang_ki', default_value='0.0')
    ang_kd_arg = DeclareLaunchArgument('ang_kd', default_value='0.0')

    max_v_arg = DeclareLaunchArgument('max_linear_speed', default_value='0.6')
    max_w_arg = DeclareLaunchArgument('max_angular_speed', default_value='1.5')

    controller_node = Node(
        package='go2_local_controller',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'lookahead_distance': LaunchConfiguration('lookahead_distance'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'yaw_tolerance': LaunchConfiguration('yaw_tolerance'),
            'lin_kp': LaunchConfiguration('lin_kp'),
            'lin_ki': LaunchConfiguration('lin_ki'),
            'lin_kd': LaunchConfiguration('lin_kd'),
            'ang_kp': LaunchConfiguration('ang_kp'),
            'ang_ki': LaunchConfiguration('ang_ki'),
            'ang_kd': LaunchConfiguration('ang_kd'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        lookahead_arg,
        goal_tol_arg,
        yaw_tol_arg,
        lin_kp_arg,
        lin_ki_arg,
        lin_kd_arg,
        ang_kp_arg,
        ang_ki_arg,
        ang_kd_arg,
        max_v_arg,
        max_w_arg,
        controller_node,
    ])
