"""
mapeo_slam_nav_bridge.launch.py  —  Mapeo SLAM en el JetAuto REAL (bridge Orin/Nano).

Version "bridge" SIN simulacion: NO lanza Gazebo, NI ros_gz_bridge, NI el spawner
jetauto_bringup, NI joy/teleop, NI RViz. El hardware (LiDAR/ruedas/IMU) lo da el contenedor
del Nano y el computo (RSP + EKF -> /odom + TF odom->base_footprint) lo da jetauto-orin.service
(orin_compute.launch.py), ambos ya corriendo. El teleop (control) y RViz se lanzan APARTE.

Reloj REAL (chrony): use_sim_time = False.

IMPORTANTE (2026-06-01): este launch usa el MISMO SLAM VALIDADO en piso que `view.launch.py`
(paquete jetauto_slam, config mapper con scan_topic=/scan directo y max_laser_range=12.0). NO
usa el config de sim de mi_proyecto_sim (max 5.0m + filtro_lidar/scan_filtered) porque eso
degradaba el mapeo real (descartaba puntos lejanos y recortaba el FOV). filtro_lidar se reserva
para la NAVEGACION (evasion), no para mapear.

Uso (en el Orin, con el entorno DDS del bridge ya cargado):
    ros2 launch mi_proyecto_sim mapeo_slam_nav_bridge.launch.py

Guardar el mapa (en otra terminal del Orin, mientras este launch corre):
    ros2 run mi_proyecto_sim guardar_mapa_slam.py --ros-args \\
        -p output_dir:=$HOME/maps -p map_name:=mapa_real

IMPORTANTE: NO lanzar a la vez `view.launch.py slam:=true` desde la laptop (doble slam_toolbox
-> conflicto de TF map->odom). Usar RViz con slam:=false.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Reutiliza EXACTAMENTE el SLAM validado en piso (jetauto_slam): config con /scan directo,
    # max_laser_range 12.0, min 0.15. start_robot:=false (el hardware ya lo da el bridge),
    # use_rviz:=false (RViz se lanza aparte desde la laptop).
    jetauto_slam_share = get_package_share_directory('jetauto_slam')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jetauto_slam_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'start_robot': 'false',
            'use_rviz': 'false',
        }.items(),
    )

    return LaunchDescription([
        slam,
    ])
