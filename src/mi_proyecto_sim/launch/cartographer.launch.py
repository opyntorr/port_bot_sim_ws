"""
Launch de Cartographer en LOCALIZATION mode con prior congelado.

Reemplaza al stack slam_toolbox + map_server del dron. El prior viene de un
.pbstream generado con tools/pgm_to_pbstream.py a partir del PGM del stitching.

Topicos:
  /scan_filtered   (input LaserScan)
  /odom            (input Odometry)
  /map             (output OccupancyGrid del grid fusionado)
  /submap_list     (output, lista de submaps activos)

TFs publicadas:
  map -> odom      (Cartographer, corrige drift con cada scan match)

Argumentos:
  pbstream_file:   ruta al .pbstream prior (default: maps/mapa_mision.pbstream)
  use_sim_time:    true para Gazebo
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')

    default_pbstream = os.path.join(pkg_sim, 'maps', 'mapa_mision.pbstream')
    default_lua_dir = os.path.join(pkg_sim, 'config')

    pbstream_arg = DeclareLaunchArgument(
        'pbstream_file',
        default_value=default_pbstream,
        description='Ruta al .pbstream prior (mapa congelado del dron)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    lua_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value='cartographer_2d_localization.lua')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', default_lua_dir,
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
            '-load_state_filename', LaunchConfiguration('pbstream_file'),
            '-load_frozen_state', 'true',
        ],
        remappings=[
            ('scan', '/scan_filtered'),
            ('odom', '/odom'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
        ],
    )

    # Relay: el planner aun escucha /map_dron. Con Cartographer+prior, /map ya
    # incluye el grid del dron + lo que vea el carrito, asi que lo reflejamos
    # tambien en /map_dron sin necesidad de map_server.
    map_relay = Node(
        package='topic_tools',
        executable='relay',
        name='map_to_map_dron_relay',
        arguments=['/map', '/map_dron'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # publicador_tfs_arucos en modo carto_frozen_prior: publica identity TF
    # map -> map_dron_origin + arucos directo en sus coords del YAML.
    publicador_tfs = Node(
        package='mi_proyecto_sim',
        executable='publicador_tfs_arucos.py',
        name='publicador_tfs_arucos',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'carto_frozen_prior': True},
        ],
    )

    return LaunchDescription([
        pbstream_arg,
        use_sim_time_arg,
        lua_basename_arg,
        cartographer_node,
        occupancy_grid_node,
        map_relay,
        publicador_tfs,
    ])
