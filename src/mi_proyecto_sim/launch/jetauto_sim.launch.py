#!/usr/bin/env python3
"""
jetauto_sim.launch.py  —  JetAuto SIMULATION (Ignition gz-sim), sim-to-real.

Trae el JetAuto real a la simulación reproduciendo su stack tal cual corre en la
Jetson Orin (ver jetauto_bringup/robot.launch.py + jetauto_controller/chassis.launch.py):

  gz (laberinto.sdf)                          mundo + física + sensores + IMU
  robot_state_publisher (jetauto_sim.xacro)   árbol TF estático (base_footprint->...->lidar_frame)
  ros_gz spawn  -name jetauto                  inserta el robot (ruedas mecanum con rodillos+fricción)
  ros2_control: joint_state_broadcaster + velocity_controller   (velocidad en las 4 ruedas)
  jetauto_chassis_sim   /cmd_vel -> 4 velocidades de rueda (mecanum.py real) + /odom_raw
  imu_filter_madgwick   /imu/data_raw -> /imu/data
  ekf (robot_localization)  /odom_raw + /imu/data -> /odom + TF odom->base_footprint

Topics/marcos IDÉNTICOS al robot real: /scan(lidar_frame), /imu/data, /odom, /cam_1/image.

USO:
  ros2 launch mi_proyecto_sim jetauto_sim.launch.py
  ros2 launch mi_proyecto_sim jetauto_sim.launch.py use_rviz:=false use_teleop:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable,
                            AppendEnvironmentVariable, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')

    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    xacro_file = os.path.join(pkg_sim, 'urdf', 'jetauto', 'jetauto_sim.urdf.xacro')
    ekf_yaml = os.path.join(pkg_sim, 'config', 'jetauto_ekf.yaml')
    teleop_yaml = os.path.join(pkg_sim, 'config', 'xbox_mecanum.yaml')
    models_dir = os.path.join(pkg_sim, 'models')

    use_rviz = LaunchConfiguration('use_rviz')
    use_teleop = LaunchConfiguration('use_teleop')
    use_soporte = LaunchConfiguration('use_soporte')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')

    args = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('use_soporte', default_value='true',
                              description='true: soporte Legera + ArUco 3D + MS200 (default). false: robot original (A1)'),
        DeclareLaunchArgument('x', default_value='-1.3'),
        DeclareLaunchArgument('y', default_value='-1.3'),
        DeclareLaunchArgument('z', default_value='0.08'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
    ]

    # --- entorno para que gz encuentre modelos/mallas y los plugins de ros2_control ---
    set_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', value=models_dir + ':' + pkg_sim)
    append_plugins = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH', value='/opt/ros/humble/lib')

    # --- Gazebo (Ignition Fortress) ---
    gz = ExecuteProcess(cmd=['ign', 'gazebo', '-r', world_file], output='screen')

    # --- puente ROS <-> gz (igual marcos/topics que el robot real) ---
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/scan_a1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/cam_1/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam_1/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cam_1/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
    )

    # --- robot_state_publisher (URDF JetAuto sim) ---
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_soporte:=', use_soporte]), value_type=str)
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
    )

    # --- spawn del robot en gz ---
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'jetauto', '-topic', 'robot_description',
                   '-x', spawn_x, '-y', spawn_y, '-z', spawn_z, '-Y', spawn_yaw],
    )

    # --- controladores ros2_control (se cargan tras el spawn) ---
    jsb = Node(package='controller_manager', executable='spawner', output='screen',
               arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    vel_ctrl = Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=['velocity_controller', '--controller-manager', '/controller_manager'])

    # --- driver fiel: /cmd_vel -> velocidad de rueda + /odom_raw (dead-reckoning) ---
    chassis_sim = Node(
        package='mi_proyecto_sim', executable='jetauto_chassis_sim.py',
        name='jetauto_chassis_sim', output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # --- IMU madgwick: /imu/data_raw -> /imu/data (orientación) ---
    madgwick = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        name='imu_filter', output='screen',
        parameters=[{'use_sim_time': True, 'use_mag': False,
                     'publish_tf': False, 'world_frame': 'enu'}],
    )

    # --- EKF robot_localization: /odom_raw + /imu/data -> /odom (+ TF odom->base_footprint) ---
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_yaml],
        remappings=[('odometry/filtered', 'odom')],
    )

    # --- teleop (joystick) opcional ---
    joy = Node(package='joy', executable='joy_node', name='joy_node',
               output='screen', condition=IfCondition(use_teleop))
    teleop = Node(
        package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy',
        parameters=[ParameterFile(teleop_yaml, allow_substs=True)],
        remappings=[('joy', '/joy'), ('cmd_vel', '/cmd_vel')],
        output='screen', condition=IfCondition(use_teleop))

    # --- RViz opcional ---
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                parameters=[{'use_sim_time': True}], condition=IfCondition(use_rviz))

    # secuencia: spawn -> joint_state_broadcaster -> velocity_controller -> chassis_sim
    seq = [
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[vel_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=vel_ctrl, on_exit=[chassis_sim])),
    ]

    return LaunchDescription(args + [
        set_resource, append_plugins,
        gz, bridge, rsp, spawn,
        madgwick, ekf,
        joy, teleop, rviz,
    ] + seq)
