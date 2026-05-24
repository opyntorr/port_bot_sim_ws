"""
Launch para construir un mapa con SLAM Toolbox controlando el carrito
manualmente con un mando Xbox.

Incluye:
  - Gazebo con el mundo del laberinto.
  - Carrito Rosmaster X3 (URDF + control PID via effort_controller).
  - Puente ROS <-> Gazebo (clock, cmd_vel, scan, odometry, tf, pose).
  - odom_noise_filter: republica /odom_raw -> /odom y /tf_raw -> /tf
    (esencial para que SLAM tenga la cadena TF odom -> base_footprint).
    Ruido en 0 por defecto, no introduce drift artificial.
  - Joy + teleop_twist_joy para control con mando Xbox.
  - filtro_lidar (genera /scan_filtered que consume SLAM Toolbox).
  - SLAM Toolbox en modo mapping (online_async) con
    config/mapper_params_online_async.yaml.
  - RViz con rviz/mapeo.rviz (display de Map, LaserScan, TF, RobotModel).

Para guardar el mapa, en otra terminal mientras la sim corre:
    ros2 run mi_proyecto_sim guardar_mapa_slam.py --ros-args \\
        -p output_dir:=src/mi_proyecto_sim/maps \\
        -p map_name:=mi_mapa
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable,
    TimerAction, IncludeLaunchDescription, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')
    xacro_file = os.path.join(pkg_sim, 'urdf', 'carrito_con_aruco_pid.urdf.xacro')
    slam_params = os.path.join(pkg_sim, 'config', 'mapper_params_online_async.yaml')
    xbox_params = os.path.join(pkg_sim, 'config', 'xbox_mecanum.yaml')

    # =========================================================
    # GAZEBO
    # =========================================================
    set_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_dir,
    )
    plugin_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib',
    )
    gui_fov_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_GUI_CAMERA_FOV',
        value='0.691',
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen',
    )

    puente = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/cam_1/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/model/rosmaster_x3/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/rosmaster_x3/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/rosmaster_x3/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose',
        ],
        remappings=[
            ('/model/rosmaster_x3/odometry', '/odom_raw'),
            ('/model/rosmaster_x3/tf', '/tf_raw'),
            ('/model/rosmaster_x3/pose', '/odom_pose'),
        ],
        output='screen',
    )

    # =========================================================
    # CARRITO ROSMASTER (URDF + PID)
    # =========================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file, ' use_gazebo:=true']),
                value_type=str,
            ),
        }],
    )

    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rosmaster_x3',
            '-topic', 'robot_description',
            '-x', '-1.0',
            '-y', '-1.0',
            '-z', '0.1',
            '-Y', '1.5708',
        ],
        output='screen',
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )

    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'effort_controller'],
        output='screen',
    )

    pid_node = Node(
        package='mi_proyecto_sim',
        executable='Mcnamu_driver_PID_sim.py',
        name='driver_node_pid',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'Kp': 6.0,
            'Ki': 2.0,
            'Kff': 9.0,
            'deadband': 32.0,
        }],
    )

    seq_1 = RegisterEventHandler(
        OnProcessExit(target_action=spawner, on_exit=[load_joint_state_broadcaster])
    )
    seq_2 = RegisterEventHandler(
        OnProcessExit(target_action=load_joint_state_broadcaster,
                      on_exit=[load_effort_controller])
    )
    seq_3 = RegisterEventHandler(
        OnProcessExit(target_action=load_effort_controller, on_exit=[pid_node])
    )

    # =========================================================
    # ODOM + TF FORWARDER
    # =========================================================
    odom_forwarder = Node(
        package='mi_proyecto_sim',
        executable='odom_noise_filter.py',
        name='odom_noise_filter',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'noise_std_x': 0.0,
            'noise_std_y': 0.0,
            'noise_std_yaw': 0.0,
            'drift_x_per_sec': 0.0,
            'drift_y_per_sec': 0.0,
            'drift_yaw_per_sec': 0.0,
        }],
    )

    # =========================================================
    # ARGUMENTOS DE LAUNCH
    # =========================================================
        # =========================================================
    # CONFIGURACION COMUN (SLAM, RViz, Teleop)
    # =========================================================
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[ParameterFile(xbox_params, allow_substs=True)],
        remappings=[
            ('joy', '/joy'),
            ('cmd_vel', '/cmd_vel'),
        ],
        output='screen',
    )

    filtro_lidar_node = Node(
        package='mi_proyecto_sim',
        executable='filtro_lidar.py',
        name='filtro_lidar',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    slam_toolbox_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch', 'online_async_launch.py'
                    )
                ),
                launch_arguments={
                    'slam_params_file': slam_params,
                    'use_sim_time': False,
                }.items(),
            )
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/ros2_ws/mi_config.rviz'],
        parameters=[{'use_sim_time': False}],
    )

    # =========================================================
    # GRUPO SIMULACION (Solo si use_sim == true)
    # =========================================================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'use_sim_time': False,
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_file, ' use_gazebo:=false']),
                    value_type=str,
                ),
            }],
        ),
        Node(
            package='yahboom_base',
            executable='base',
            name='yahboom_base',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('yahboomcar_ydlidar'),
                    'launch', 'ydlidar_launch.py'
                )
            ),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),
    ])
