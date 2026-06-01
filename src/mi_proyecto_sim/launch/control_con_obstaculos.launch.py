import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    obstaculos_sdf = os.path.join(pkg_sim, 'models', 'obstaculos_var', 'model.sdf')

    # 1. Spawnear el mesh de obstaculos variables (obstaculosVar.stl) en Gazebo.
    #    Asume que Gazebo ya esta corriendo (lanzado por simulacion.launch.py).
    #    Pose identica a la del modelo `laberinto_real` en laberinto.sdf:
    #      <pose>-2.294 2.294 0 1.5708 0 0</pose>  (traslacion + roll=pi/2).
    spawn_obstaculos = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_obstaculos_var',
        arguments=[
            '-name', 'obstaculos_var',
            '-file', obstaculos_sdf,
            '-x', '-2.294',
            '-y', '2.294',
            '-z', '0.0',
            '-R', '1.5708',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # 2. Nodo de control de trayectoria — se lanza DESPUES de que termine el spawn.
    control_trayectoria_node = Node(
        package='mi_proyecto_sim',
        executable='control_trayectoria.py',
        name='control_trayectoria',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Pequeño retardo extra para asegurar que la fisica del mesh este cargada
    # antes de que el LiDAR / planificador empiecen a reaccionar a el.
    control_diferido = TimerAction(period=1.5, actions=[control_trayectoria_node])

    # Handler: cuando el spawner termina su trabajo, arrancamos el control.
    arrancar_control = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_obstaculos,
            on_exit=[control_diferido],
        )
    )

    return LaunchDescription([
        spawn_obstaculos,
        arrancar_control,
    ])
