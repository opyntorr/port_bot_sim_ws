import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    # Derivar la raíz del workspace: install/mi_proyecto_sim/share/mi_proyecto_sim -> 4 niveles arriba
    ws_root = os.path.abspath(os.path.join(pkg_sim, '..', '..', '..', '..'))
    mapa_pgm = os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'maps', 'mapa_mision.pgm')
    # mapa_yaml = os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'maps', 'mapa_laberinto.yaml')
    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')
    xacro_file = os.path.join(pkg_sim, 'urdf', 'carrito_con_aruco.urdf.xacro')
    rviz_config = os.path.join(pkg_sim, 'rviz', 'configuracion.rviz')


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
            ('/model/rosmaster_x3/odometry', '/odom'),
            ('/model/rosmaster_x3/tf', '/tf'),
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

    # TF estática por defecto: map → odom (identidad).
    # control_trayectoria la sobreescribirá con la calibración ArUco cuando arranque.
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # 6. Servidor de Mapas (Nav2 Map Server) — COMENTADO: reemplazado por slam_occupancy_grid
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{
    #         'yaml_filename': mapa_yaml,
    #         'use_sim_time': True
    #     }]
    # )

    # 7. Administrador de Ciclo de Vida — COMENTADO: ya no se necesita sin map_server
    # lifecycle_manager_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_map',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'autostart': True,
    #         'node_names': ['map_server']
    #     }]
    # )

    # 8. Filtro LiDAR (Software 190 grados)
    filtro_lidar_node = Node(
        package='mi_proyecto_sim',
        executable='filtro_lidar.py',
        name='filtro_lidar',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 9. SLAM Occupancy Grid (Fusion Dron + LiDAR con replanificacion)
    # pgm_resolution y pgm_origin se leen del YAML automaticamente;
    # los valores aqui son fallback si el YAML no existe.
    slam_occupancy_grid_node = Node(
        package='mi_proyecto_sim',
        executable='slam_occupancy_grid.py',
        name='slam_occupancy_grid',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pgm_path': mapa_pgm,
            'pgm_resolution': 0.002,
            'pgm_origin_x': -3.35,
            'pgm_origin_y': -3.84,
            'map_resolution': 0.05,
            'map_width': 160,
            'map_height': 180,
            'map_origin_x': -4.0,
            'map_origin_y': -4.5,
            'map_to_odom_yaw': 0.0,
        }]
    )

    # 10. SLAM Toolbox — COMENTADO: reemplazado por slam_occupancy_grid
    # slam_node = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'odom_frame': 'map',
    #         'base_frame': 'base_footprint',
    #         'map_frame': 'map_slam',
    #         'scan_topic': '/scan_filtered',
    #         'mode': 'mapping'
    #     }],
    #     remappings=[('/map', '/map_slam')]
    # )

    # Nodo de Planificación de Ruta
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

    # Nodo que publica TFs estaticas de ArUcos (lee arucos.yaml generado por mision_dron)
    publicador_tfs_node = Node(
        package='mi_proyecto_sim',
        executable='publicador_tfs_arucos.py',
        name='publicador_tfs_arucos',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Event handler: cuando mision_dron termina, lanzar SLAM + TF publisher + RRT
    rrt_y_slam_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mision_dron_node,
            on_exit=[
                publicador_tfs_node,
                slam_occupancy_grid_node,
                planificador_rrt_node
            ]
        )
    )

    # Empaquetar y lanzar todo simultaneamente
    return LaunchDescription([
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
        static_map_to_odom,          # Puente TF: map -> odom (identidad por defecto)
        filtro_lidar_node,
        optitrack_sim,
        pose_fuser,
        pid_controller,
        plotter,
        mision,
        rrt_y_slam_handler,
    ])

