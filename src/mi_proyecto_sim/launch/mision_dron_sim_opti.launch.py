"""
Mision autonoma del dron en SIMULACION usando OptiTrack simulado (ground truth
de Gazebo) como unica fuente de pose+orientacion, con fallback a /drone1/odom:

  - optitrack_simulator (con publish_orientation=True) reescribe /drone1/odom
    como /drone_pose con pose+orientacion reales del modelo Gazebo.
  - pose_fuser_optitrack consume /drone_pose y publica /odometry/filtered.
  - position_controller con control de yaw habilitado.
  - mision_dron en modo SIM + stitching por pose.

Asume que Gazebo + el spawner del Tello ya estan corriendo
(p. ej. con `ros2 launch mi_proyecto_sim simulacion.launch.py`).

Uso:
  ros2 launch mi_proyecto_sim mision_dron_sim_opti.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    ws_root = os.path.abspath(os.path.join(pkg_sim, '..', '..', '..', '..'))

    stitch_script = os.path.join(
        ws_root, 'src', 'mi_proyecto_sim', 'mi_proyecto_sim', 'stitching_pose.py'
    )
    mision_output = os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'mision_output')
    camera_yaml = '/ros2_ws/src/mi_proyecto_sim/config/camera_tello_sim.yaml'

    optitrack_sim = Node(
        package='tello_control_pos',
        executable='optitrack_simulator',
        name='optitrack_simulator',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'latency_sec': 0.005},
            {'publish_orientation': True},
        ],
    )

    pose_fuser = Node(
        package='tello_control_pos',
        executable='pose_fuser_optitrack',
        name='pose_fuser_optitrack',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    pid_controller = Node(
        package='tello_control_pos',
        executable='position_controller',
        name='position_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'velocity_scale': 1.0},
            {'kp': 0.4},
            {'ki': 0.02},
            {'kd': 0.4},
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
        parameters=[{'use_sim_time': True}],
    )

    mision_node = Node(
        package='mi_proyecto_sim',
        executable='mision_dron.py',
        name='mision_dron',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'use_real_drone': False},
            {'camera_topic': '/uav/camera/image'},
            {'odom_topic': '/odometry/filtered'},
            {'stitcher': 'pose'},
            {'stitch_resolution': 0.005},
            {'camera_yaml': camera_yaml},
            {'photo_interval': 0.2},
            {'photo_z_tol': 0.3},
        ],
    )

    mision = TimerAction(period=5.0, actions=[mision_node])

    stitching_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mision_node,
            on_exit=[ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    f'FOTOS=$(ls -td {mision_output}/*/fotos 2>/dev/null | head -1) && '
                    f'[ -n "$FOTOS" ] && '
                    f'python3 {stitch_script} '
                    f'--input "$FOTOS" '
                    f'--output "${{FOTOS%/fotos}}/stitching" '
                    f'--camera {camera_yaml} '
                    f'--resolution 0.005 || '
                    f'echo "[stitching] No se encontro directorio de fotos"'
                ],
                output='screen',
            )],
        )
    )

    return LaunchDescription([
        optitrack_sim,
        pose_fuser,
        pid_controller,
        plotter,
        mision,
        stitching_handler,
    ])
