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
        value=models_dir + ':' + pkg_sim,  # pkg_sim: mallas del JetAuto
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

    # Puente: solo /clock. Sensores del robot (/scan, /cam_1/*, /imu/data_raw) y odom/TF
    # los aporta jetauto_bringup. /cmd_vel lo consume jetauto_chassis_sim (lado ROS).
    puente = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )

    # =========================================================
    # ROBOT JETAUTO (bringup reutilizable)
    # =========================================================
    jetauto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'jetauto_bringup.launch.py')
        ),
        launch_arguments={
            'x': '-1.0', 'y': '-1.0', 'z': '0.08', 'yaw': '1.5708',
            'use_sim_time': 'true',
        }.items(),
    )

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
                    'use_sim_time': 'true',
                }.items(),
            )
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_sim, 'rviz', 'mi_config.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    # =========================================================
    # LANZAR TODO (robot via include jetauto + SLAM + teleop + RViz)
    # =========================================================
    return LaunchDescription([
        set_env,
        plugin_env,
        gui_fov_env,
        gazebo,
        puente,
        jetauto,
        joy_node,
        teleop,
        slam_toolbox_launch,
        rviz_node,
    ])
