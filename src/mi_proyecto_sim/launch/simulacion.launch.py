import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')
    xacro_file = os.path.join(pkg_sim, 'urdf', 'carrito_con_aruco.urdf.xacro')
    rviz_config = os.path.join(pkg_sim, 'rviz', 'configuracion.rviz')

    # 1. Variables de entorno para que Gazebo encuentre los modelos
    set_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_dir
    )

    # Añadir la ruta de los plugins de ROS 2 a Gazebo
    plugin_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    # 2. Lanzar Gazebo sin pausa
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )

    # 3. Lanzar el Puente (ros_gz_bridge)
    puente = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock', # <--- ¡EL RELOJ!
            '/uav/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/cam_1/image@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        output='screen'
    )

    # Visor de la cámara del dron (Ahora sintonizado al canal con ArUco)
    visor_dron = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='visor_dron',
        arguments=['/uav/camera/aruco_3d']  # <--- Esta es la clave mágica
    )

    # 4b. NUEVO: Lanzar el visor de imágenes para el Carrito
    visor_carrito = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view_carrito', # Nombre único
        arguments=['/cam_1/image'], # El tópico de la cámara de tu carrito
        output='screen'
    )

    # =========================================================
    # NODOS PARA EL CARRITO ROSMASTER
    # =========================================================

    # 5. Robot State Publisher (Traduce el Xacro y lo publica)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,  # <--- Sincronizado
            'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' use_gazebo:=true']), value_type=str)
        }]
    )

    # 6. Gazebo Spawner
    spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rosmaster_x3',
            '-topic', 'robot_description',
            
            # --- POSICIÓN ---
            '-x', '-1.0',  # Coordenada X en metros
            '-y', '1.0',  # Coordenada Y en metros
            '-z', '0.1',  # Altura inicial
            
            # --- ORIENTACIÓN (Añadir esto) ---
            '-Y', '0' # Orientación (Yaw) en radianes. 
        ],
        output='screen'
    )

    # Visor RViz con configuración guardada
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/ros2_ws/mi_config.rviz'],
        parameters=[{'use_sim_time': True}]  # <--- Sincronizado
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[
            ParameterFile(
                os.path.join(pkg_sim, 'config', 'xbox_mecanum.yaml'),
                allow_substs=True
            )
        ],
        remappings=[
            ('joy', '/joy'),
            ('cmd_vel', '/cmd_vel')
        ],
        output='screen'
    )

    # Nodo del Cerebro 3D (Detector ArUco y TF2)
    detector_aruco_node = Node(
        package='mi_proyecto_sim',
        executable='detector_aruco.py',
        name='detector_aruco',
        output='screen',
        parameters=[{'use_sim_time': True}]  # <--- ¡DESCOMENTADO! Fundamental para las coordenadas 3D
    )

    # Nodo para conectar el ArUco con el modelo 3D del carrito
    tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='aruco_to_robot_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'carrito_aruco', 'base_footprint']
    )

    # 6. Servidor de Mapas (Nav2 Map Server)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': '/ros2_ws/mapa_laberinto.yaml',
            'use_sim_time': True  # <--- ¡DESCOMENTADO! Sincronizado con Gazebo
        }]
    )

    # 7. Administrador de Ciclo de Vida (Para despertar al Map Server)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True, # <--- ¡DESCOMENTADO!
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # Empaquetar y lanzar todo simultáneamente
    return LaunchDescription([
        set_env,
        plugin_env,
        gazebo,
        puente,
        visor_dron,
        visor_carrito,
        robot_state_publisher,
        spawner,
        rviz_node,
        joy_node,
        teleop,
        detector_aruco_node,
        map_server_node,
        lifecycle_manager_node
    ])