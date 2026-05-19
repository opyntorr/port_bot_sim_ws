"""
Mision autonoma del dron Tello REAL con OptiTrack como unica fuente de pose
y orientacion (con fallback de odometria):
  - Driver Tello (WiFi 192.168.10.1)
  - pose_fuser_optitrack (posicion + orientacion del optitrack; fallback /drone1/odom)
  - position_controller con control de yaw habilitado (yaw_ref = primer dato)
  - plotter
  - mision_dron en modo real + stitching por pose

Prerrequisitos:
  - PC conectada al WiFi del Tello
  - OptiTrack publicando en /optitrack/rigid_body (pose + orientacion)

Uso:
  ros2 launch mi_proyecto_sim mision_dron_real_opti.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            {'camera_info_file': '/ros2_ws/src/demo_tello_sim/src/tello-ros2-main/workspace/src/tello/resource/ost.yaml'},
        ],
        remappings=[
            ('image_raw', '/image_raw'),
            ('odom', '/odom'),
            ('control', '/control'),
            ('takeoff', '/takeoff'),
            ('land', '/land'),
        ],
    )

    pose_fuser = Node(
        package='tello_control_pos',
        executable='pose_fuser_optitrack',
        name='pose_fuser_optitrack',
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
            {'kp': 0.5},
            {'ki': 0.06},
            {'kd': 0.35},
            {'enable_yaw_control': True},
            {'kp_yaw': 1.5},
            {'kd_yaw': 0.15},
            {'max_yaw_rate': 0.8},
        ],
    )

    plotter = Node(
        package='tello_control_pos',
        executable='plotter',
        name='plotter',
        output='screen',
        parameters=[{'use_sim_time': False}],
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
                {'stitcher': 'pose'},
                {'stitch_resolution': 0.005},
                # Real: el Tello ya produce colores "invertidos" de fábrica, no añadir swap
                {'invert_colors': False},
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
