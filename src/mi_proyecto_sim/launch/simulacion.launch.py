import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

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
            # Dron (El que ya tenías)
            '/uav/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            
            # Llantas del Carrito
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            
            # LiDAR del Carrito (De Gazebo a ROS 2)
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            
            # Cámara del Carrito (De Gazebo a ROS 2)
            '/cam_1/image@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        output='screen'
    )

    # 4. Lanzar el visor de imágenes (rqt)
    visor = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/uav/camera/image'],
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
            # Añadimos use_gazebo:=true para cargar las llantas Mecanum en el simulador
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
                           # 1.5708 rad = 90 grados
        ],
        output='screen'
    )

    # 7. Lanzar RViz2 con la configuración guardada
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config], # El parámetro -d le dice qué archivo cargar
        output='screen'
    )

    # Empaquetar y lanzar todo simultáneamente
    return LaunchDescription([
        set_env,
        plugin_env,
        gazebo,
        puente,
        visor,
        robot_state_publisher,
        spawner,
        rviz_node
    ])