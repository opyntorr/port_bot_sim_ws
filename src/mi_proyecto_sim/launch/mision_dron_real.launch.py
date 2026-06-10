"""
Misión autónoma del dron Tello REAL:
  - Driver Tello (WiFi 192.168.10.1)
  - pose_fuser (fusión OptiTrack + odometría)
  - PID de posición
  - mision_dron (modo real)

Prerrequisitos:
  - PC conectada al WiFi del Tello
  - OptiTrack publicando en /optitrack/rigid_body

Uso:
  ros2 launch mi_proyecto_sim mision_dron_real.launch.py
  ros2 launch mi_proyecto_sim mision_dron_real.launch.py tello_ip:=192.168.10.1
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_TELLO_SHARE = Path(get_package_share_directory('tello'))
_OST_YAML = str(_TELLO_SHARE / 'ost.yaml')


def generate_launch_description():
    tello_ip_arg = DeclareLaunchArgument(
        'tello_ip', default_value='192.168.10.1',
        description='IP del Tello en la red WiFi'
    )
    tello_ip = LaunchConfiguration('tello_ip')

    driver = Node(
        package='tello',
        executable='tello',
        name='tello_driver',
        output='screen',
        parameters=[
            {'tello_ip': tello_ip},
            {'camera_info_file': _OST_YAML}
        ],
        remappings=[
            ('image_raw', '/image_raw'),
            ('odom', '/odom'),
            ('control', '/control'),
            ('takeoff', '/takeoff'),
            ('land', '/land'),
            ('flight_state', '/flight_state'),
        ]
    )

    pose_fuser = Node(
        package='tello_control_pos',
        executable='pose_fuser',
        name='pose_fuser',
        output='screen',
        remappings=[
            ('/drone1/odom', '/odom'),
            ('/drone_pose', '/optitrack/rigid_body'),
        ],
        parameters=[{'use_sim_time': False}],
    )

    pid_controller = Node(
        package='tello_control_pos',
        executable='position_controller',
        name='position_controller',
        output='screen',
        remappings=[
            ('/drone1/cmd_vel', '/control'),
            ('/odometry/filtered', '/odometry/filtered'),
        ],
        parameters=[
            {'use_sim_time': False},
            {'velocity_scale': 100.0},
            {'kp': 0.4},
            {'ki': 0.15},
            {'ki_z': 0.25},
            {'kd': 0.35},
            {'use_flight_state': True},
            {'cmd_pub_rate': 20.0},
        ],
    )

    plotter = Node(
        package='tello_control_pos',
        executable='plotter',
        name='plotter',
        output='screen',
        parameters=[{'use_sim_time': False}],
        additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')},
    )

    land_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['ros2', 'topic', 'pub', '--once', '/land', 'std_msgs/msg/Empty', '{}'],
                    output='screen',
                )
            ]
        )
    )

    # La misión arranca 5s después del driver para que el Tello termine de conectarse
    mision = TimerAction(
        period=5.0,
        actions=[Node(
            package='mi_proyecto_sim',
            executable='mision_dron.py',
            name='mision_dron',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'use_real_drone': True},
            ],
        )],
    )

    return LaunchDescription([
        tello_ip_arg,
        driver,
        pose_fuser,
        pid_controller,
        plotter,
        land_on_shutdown,
        mision,
    ])
