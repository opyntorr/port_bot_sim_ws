import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable, RegisterEventHandler, TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')

    # Backend de SLAM: 'slam_toolbox' (default, comportamiento original) o
    # 'cartographer' (Cartographer en localization mode con prior del dron).
    slam_backend_arg = DeclareLaunchArgument(
        'slam_backend',
        default_value='slam_toolbox',
        choices=['slam_toolbox', 'cartographer'],
        description='Backend de SLAM a usar')
    is_slam_toolbox = LaunchConfigurationEquals('slam_backend', 'slam_toolbox')
    is_cartographer = LaunchConfigurationEquals('slam_backend', 'cartographer')
    # Derivar la raíz del workspace: install/mi_proyecto_sim/share/mi_proyecto_sim -> 4 niveles arriba
    ws_root = os.path.abspath(os.path.join(pkg_sim, '..', '..', '..', '..'))
    maps_dir = os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'maps')
    mapa_pgm = os.path.join(maps_dir, 'mapa_mision.pgm')
    mapa_yaml = os.path.join(maps_dir, 'mapa_mision.yaml')
    arucos_yaml = os.path.join(maps_dir, 'arucos.yaml')
    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')
    xacro_file = os.path.join(pkg_sim, 'urdf', 'carrito_con_aruco_pid.urdf.xacro')
    rviz_config = os.path.join(pkg_sim, 'rviz', 'configuracion.rviz')

    # Limpiar artefactos de la corrida anterior antes de lanzar.
    # Asi, si la mision del dron crashea, slam_occupancy_grid no carga el PGM
    # viejo y RViz queda en blanco (solo_carrito.launch.py NO hace esto:
    # ahi si queremos reutilizar los artefactos).
    limpiar_artefactos = ExecuteProcess(
        cmd=['rm', '-f', mapa_pgm, mapa_yaml, arucos_yaml],
        output='screen',
    )


    tello_models_dir = os.path.join(ws_root, 'src', 'demo_tello_sim', 'src', 'tello-ros2-gazebo-master', 'tello_ros', 'tello_gazebo', 'models')
    tello_espejo_sdf = os.path.join(models_dir, 'tello_con_espejo', 'model.sdf')

    # 1. Variables de entorno para que Gazebo encuentre los modelos
    set_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=f"{models_dir}:{tello_models_dir}"
    )

    tello_lib_dir = os.path.join(ws_root, 'install', 'tello_gazebo', 'lib')
    
    # Añadir la ruta de los plugins de ROS 2 a Gazebo
    plugin_env = AppendEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=f"/opt/ros/humble/lib:{tello_lib_dir}"
    )

    # FOV del viewport (~50mm en full-frame ≈ 0.691 rad). No todas las
    # versiones de Ignition Fortress respetan esta var; si no funciona, no rompe nada.
    gui_fov_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_GUI_CAMERA_FOV',
        value='0.691'
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
            '/uav/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image', # Camara del espejo del dron
            '/drone1/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist', # Control del dron
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/cam_1/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/model/rosmaster_x3/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/rosmaster_x3/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/rosmaster_x3/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose'
        ],
        remappings=[
            ('/model/rosmaster_x3/odometry', '/odom_raw'),
            ('/model/rosmaster_x3/tf', '/tf_raw'),
            ('/model/rosmaster_x3/pose', '/odom_pose')
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
            '-y', '-1.0',  # Coordenada Y en metros
            '-z', '0.1',  # Altura inicial
            
            # --- ORIENTACIÓN (Añadir esto) ---
            '-Y', '1.5708' # Orientación (Yaw) en radianes. 
        ],
        output='screen'
    )

    # =========================================================
    # CONTROL PID PARA SIMULACION (Effort Controller)
    # =========================================================
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controller'],
        output='screen'
    )

    pid_node = Node(
        package='mi_proyecto_sim',
        executable='Mcnamu_driver_PID_sim.py',
        name='driver_node_pid',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'Kp': 6.0,
            'Ki': 2.0,
            'Kff': 9.0,
            'deadband': 32.0
        }]
    )

    seq_1 = RegisterEventHandler(
        OnProcessExit(target_action=spawner, on_exit=[load_joint_state_broadcaster])
    )
    seq_2 = RegisterEventHandler(
        OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_effort_controller])
    )
    seq_3 = RegisterEventHandler(
        OnProcessExit(target_action=load_effort_controller, on_exit=[pid_node])
    )


    # 6b. Gazebo Spawner Dron
    spawner_dron = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'drone1',
            '-file', tello_espejo_sdf,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
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
        parameters=[{
            'use_sim_time': True,
            'aruco_size_m': 0.11,
            'tamano_pixel_mapa': 440,
            'ancho_laberinto_m': 2.65,
            'alto_laberinto_m': 3.10,
            'invert_colors': True,  # sim: emula la inversión nativa del Tello real
        }]
    )


    # 6. Servidor de Mapas (Nav2 Map Server) para visualizar el mapa del dron
    # frame_id='map_dron_origin' lo conecta a `map` (SLAM) via TF estatica
    # publicada por publicador_tfs_arucos.
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': mapa_yaml,
            'use_sim_time': True,
            'frame_id': 'map_dron_origin',
        }],
        remappings=[('/map', '/map_dron')]  # Publicar en /map_dron para no chocar con SLAM Toolbox
    )

    # 7. Administrador de Ciclo de Vida para arrancar el map_server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # 8. Filtro LiDAR (Software 190 grados)
    filtro_lidar_node = Node(
        package='mi_proyecto_sim',
        executable='filtro_lidar.py',
        name='filtro_lidar',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # SLAM Toolbox oficial - usa online_async_launch.py con yaml dedicado
    # (patron tomado de dinav2/warehouse_bot, mantiene scan_topic=/scan_filtered)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_sim, 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'true',
        }.items(),
        condition=is_slam_toolbox,
    )

    # Cartographer en localization mode con prior congelado del PGM del dron.
    # Incluye su propio publicador_tfs_arucos en modo carto_frozen_prior y un
    # relay /map -> /map_dron para que el planner siga viendo el mapa.
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'pbstream_file': os.path.join(maps_dir, 'mapa_mision.pbstream'),
            'use_sim_time': 'true',
        }.items(),
        condition=is_cartographer,
    )# Nodo de Planificación de Ruta
    # planificador_node = Node(
    #     package='mi_proyecto_sim',
    #     executable='planificador_astar.py',
    #     name='planificador_astar',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    # Nodos para la Mision del Dron
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

    mision_dron_node = Node(
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
            {'camera_yaml': '/ros2_ws/src/mi_proyecto_sim/config/camera_tello_sim.yaml'},
            {'invert_colors': True},
            {'map_size_m': 3.9},
        ],
    )
    
    mision = TimerAction(
        period=5.0,
        actions=[mision_dron_node],
    )

    # Nodo de Planificacion RRT
    planificador_rrt_node = Node(
        package='mi_proyecto_sim',
        executable='planificador_rrt',
        name='planificador_rrt',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_radius_m': 0.19,
        }]
    )

    # Nodo que publica TFs estaticas de ArUcos (lee arucos.yaml generado por mision_dron).
    # Solo activo con slam_toolbox: cartographer.launch.py lanza su propia instancia
    # en modo carto_frozen_prior.
    publicador_tfs_node = Node(
        package='mi_proyecto_sim',
        executable='publicador_tfs_arucos.py',
        name='publicador_tfs_arucos',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=is_slam_toolbox,
    )

    # =========================================================
    # PUENTES PARA LA CONSOLA WEB (Seaport Console)
    # =========================================================

    # rosbridge_server: pub/sub de topics y servicios desde el navegador (port 9090).
    # Permite a la consola leer /odom, /cmd_vel, ArUco TFs, etc.
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'use_sim_time': True, 'port': 9090}],
        output='screen',
    )

    # foxglove_bridge: alimenta el panel "Visualization" de la consola (port 8765).
    # Mucho mas eficiente que rosbridge para point clouds, mapas, TFs y paths.
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'use_sim_time': True,
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10_000_000,
        }],
        output='screen',
    )

    # web_video_server: sirve los topics de imagen como MJPEG por HTTP (port 8080).
    # El panel de camara de la consola lo usa en vez de codificar frames base64.
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{'port': 8080}],
        output='screen',
    )

    convertir_mapa = ExecuteProcess(
        cmd=['python3', os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'tools', 'pgm_to_pbstream.py'),
             '--pgm', mapa_pgm,
             '--yaml', mapa_yaml,
             '--out', os.path.join(maps_dir, 'mapa_mision.pbstream')],
        output='screen'
    )

    handler_mision = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mision_dron_node,
            on_exit=[convertir_mapa]
        )
    )

    # Event handler: cuando la conversion termina, lanzar SLAM + TF publisher + RRT.
    # slam_toolbox_launch, cartographer_launch, y publicador_tfs_node se filtran
    # automaticamente por su condition (solo arranca el que corresponde al backend).
    rrt_y_slam_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=convertir_mapa,
            on_exit=[
                map_server_node,
                lifecycle_manager_node,
                publicador_tfs_node,
                slam_toolbox_launch,
                cartographer_launch,
                planificador_rrt_node
            ]
        )
    )

    odom_noise_node = Node(
        package='mi_proyecto_sim',
        executable='odom_noise_filter.py',
        name='odom_noise_filter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'noise_std_x': 0.002,
            'noise_std_y': 0.002,
            'noise_std_yaw': 0.001,
            'drift_x_per_sec': 0.001,
            'drift_y_per_sec': -0.0005,
            'drift_yaw_per_sec': 0.0005,
        }],
    )

    # Empaquetar y lanzar todo simultaneamente
    return LaunchDescription([
        slam_backend_arg,
        limpiar_artefactos,
        set_env,
        plugin_env,
        gui_fov_env,
        gazebo,
        puente,
        visor_dron,
        visor_carrito,
        robot_state_publisher,
        spawner,
        spawner_dron,
        rviz_node,
        joy_node,
        teleop,
        detector_aruco_node,

        filtro_lidar_node,
        optitrack_sim,
        pose_fuser,
        pid_controller,
        plotter,
        mision,
        handler_mision,
        rrt_y_slam_handler,
        rosbridge_server,
        foxglove_bridge_node,
        web_video_server_node,
        odom_noise_node,
        seq_1,
        seq_2,
        seq_3,
    ])

