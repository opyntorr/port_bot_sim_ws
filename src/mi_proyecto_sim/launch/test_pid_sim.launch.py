import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    ws_root = os.path.abspath(os.path.join(pkg_sim, '..', '..', '..', '..'))
    
    # 1. Variables de entorno (Modelos de Gazebo)
    models_dir = os.path.join(pkg_sim, 'models')
    tello_models_dir = os.path.join(ws_root, 'src', 'demo_tello_sim', 'src', 'tello-ros2-gazebo-master', 'tello_ros', 'tello_gazebo', 'models')
    set_env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', f"{models_dir}:{tello_models_dir}")
    plugin_env = AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', "/opt/ros/humble/lib")

    # 2. Gazebo (Mundo vacío o tu laberinto)
    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', world_file], output='screen')

    # 3. Robot State Publisher usando el NUEVO xacro
    xacro_file = os.path.join(pkg_sim, 'urdf', 'carrito_con_aruco_pid.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' use_gazebo:=true']), value_type=str)
        }]
    )

    # 4. Spawner del Carrito
    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rosmaster_x3',
            '-topic', 'robot_description',
            '-x', '-1.0', '-y', '-1.0', '-z', '0.1', '-Y', '1.5708'
        ],
        output='screen'
    )

    # 5. Puente de Gazebo para el Reloj y otros tópicos
    puente = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/rosmaster_x3/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        output='screen'
    )

    # 6. Spawners de Controladores de ROS 2 (Se activan DESPUÉS de instanciar el robot)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controller'],
        output='screen'
    )

    # 7. Tu Nodo Original de PID (Engañado por SunriseRobotLib.py)
    pid_node = Node(
        package='mi_proyecto_sim',
        executable='Mcnamu_driver_PID_sim.py',
        name='driver_node_pid',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'Kp': 6.0,      # Ganancia real
            'Ki': 2.0,      # Ganancia real
            'Kff': 9.0,     # Ganancia real
            'deadband': 32.0 # Banda muerta real
        }]
    )

    # Secuencia: Gazebo Spawner -> Broadcaster -> Effort -> PID Node
    seq_1 = RegisterEventHandler(
        OnProcessExit(target_action=spawner, on_exit=[load_joint_state_broadcaster])
    )
    seq_2 = RegisterEventHandler(
        OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_effort_controller])
    )
    seq_3 = RegisterEventHandler(
        OnProcessExit(target_action=load_effort_controller, on_exit=[pid_node])
    )

    return LaunchDescription([
        set_env, plugin_env,
        gazebo,
        robot_state_publisher,
        spawner,
        puente,
        seq_1, seq_2, seq_3
    ])
