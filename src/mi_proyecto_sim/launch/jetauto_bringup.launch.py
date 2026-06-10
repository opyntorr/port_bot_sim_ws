#!/usr/bin/env python3
"""
jetauto_bringup.launch.py  —  Bringup REUTILIZABLE del robot JetAuto (sim Ignition).

Extrae el "bloque de robot" de jetauto_sim.launch.py para incluirlo desde cualquier
launch que YA arranque Gazebo y el puente de /clock. Sustituye el viejo bringup del
rosmaster_x3 (RSP carrito + effort_controller + Mcnamu_driver_PID_sim + odom_noise_filter).

POSEE (lo que aporta este include):
  - parameter_bridge de sensores del robot (/scan, /imu/data_raw, /cam_1/*)   [jetauto_sensor_bridge]
  - robot_state_publisher (jetauto_sim.urdf.xacro)        arbol TF base_footprint->...->lidar_frame
  - ros_gz spawn  -name jetauto                            inserta el robot mecanum
  - ros2_control: joint_state_broadcaster + velocity_controller
  - jetauto_chassis_sim   /cmd_vel -> 4 velocidades de rueda (mecanum real) + /odom_raw (sin usar)
  - odometria NATIVA de gz (plugin OdometryPublisher) -> /odom + TF odom->base_footprint,
    puenteada en robot_bridge. Sustituye a imu_filter_madgwick + EKF (se desincronizaban con
    RTF bajo; el bloque comentado mas abajo explica como volver al EKF).

NO POSEE (lo mantiene cada parent que incluye este archivo):
  - el proceso de Gazebo (ign gazebo ...)         - el puente de /clock (y topics del dron)
  - las variables de entorno IGN_GAZEBO_*         - RViz, teleop, joy
  - SLAM, map_server, planificador, ArUcos, nodos del dron

REQUISITOS del parent:
  1. Arrancar gz.  2. Puentear /clock.  3. IGN_GAZEBO_RESOURCE_PATH debe incluir el share
  del paquete (pkg_sim) para hallar las mallas del jetauto.  4. NO puentear /scan, /cam_1/*,
  /model/* ni /cmd_vel->gz (cmd_vel lo consume jetauto_chassis_sim en el lado ROS).

USO:
  from launch.actions import IncludeLaunchDescription
  from launch.launch_description_sources import PythonLaunchDescriptionSource
  jetauto = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'jetauto_bringup.launch.py')),
      launch_arguments={'x': '-1.0', 'y': '-1.0', 'yaw': '1.5708'}.items())
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    xacro_file = os.path.join(pkg_sim, 'urdf', 'jetauto', 'jetauto_sim.urdf.xacro')
    ekf_yaml = os.path.join(pkg_sim, 'config', 'jetauto_ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')
    publish_sim_odom = LaunchConfiguration('publish_sim_odom_tf')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='-1.0'),
        DeclareLaunchArgument('y', default_value='-1.0'),
        DeclareLaunchArgument('z', default_value='0.08'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        # true (default): la odometria (/odom + TF odom->base_footprint) la da gz nativamente.
        # false: NO se publica esa odom -> para odometria solo-LiDAR (rf2o provee odom->base).
        DeclareLaunchArgument('publish_sim_odom_tf', default_value='true'),
    ]

    # --- puente de SENSORES del robot (nombre propio para no colisionar con el del parent) ---
    sensor_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='jetauto_sensor_bridge', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/cam_1/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cam_1/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cam_1/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
    )

    # --- puente de ODOMETRIA NATIVA de gz (plugin OdometryPublisher en jetauto_control.urdf.xacro):
    #     /odom y la TF odom->base_footprint directo del simulador (robusto ante RTF bajo).
    #     Condicional: con publish_sim_odom_tf:=false NO arranca -> odom solo-LiDAR (rf2o). ---
    odom_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='jetauto_odom_bridge', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/model/jetauto/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/jetauto/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        remappings=[
            ('/model/jetauto/odometry', '/odom'),
            ('/model/jetauto/tf', '/tf'),
        ],
        condition=IfCondition(publish_sim_odom),
    )

    # --- robot_state_publisher (URDF JetAuto sim) ---
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
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
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- ODOMETRIA: la da gz directamente (plugin OdometryPublisher en jetauto_control.urdf.xacro)
    #     -> /odom + TF odom->base_footprint, puenteados arriba en robot_bridge.
    #     Antes esto lo construia imu_filter_madgwick + EKF (fusion de /odom_raw dead-reckon + IMU),
    #     pero esa cadena se desincronizaba con el RTF bajo de la fisica de 48 rodillos -> SLAM
    #     perdia odom->base_footprint ("unconnected trees" / "lidar invalido"). El chassis_sim se
    #     mantiene (convierte /cmd_vel en velocidades de rueda); su /odom_raw queda sin usar.
    #     Para volver al EKF: restaurar madgwick+ekf, devolverlos al return, y quitar del
    #     robot_bridge los entries /model/jetauto/odometry y /model/jetauto/tf.

    # secuencia: spawn -> joint_state_broadcaster -> velocity_controller -> chassis_sim
    seq = [
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[vel_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=vel_ctrl, on_exit=[chassis_sim])),
    ]

    return LaunchDescription(args + [sensor_bridge, odom_bridge, rsp, spawn] + seq)
