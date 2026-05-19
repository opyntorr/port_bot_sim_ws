#!/usr/bin/env python3
"""
Nodo que lee arucos.yaml (generado por mision_dron) y publica TFs estaticas
para carrito_aruco y meta_aruco. Esto permite que planificador_rrt encuentre
start/goal sin necesidad de deteccion en vivo desde el dron.
"""
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def _find_maps_dir():
    docker = Path('/ros2_ws')
    if docker.exists():
        ws = docker
    else:
        ws = Path(get_package_share_directory('mi_proyecto_sim')).parents[3]
    return ws / 'src' / 'mi_proyecto_sim' / 'maps'


class PublicadorTfsArucos(Node):
    def __init__(self):
        super().__init__('publicador_tfs_arucos')

        self.declare_parameter('arucos_yaml', '')

        yaml_param = self.get_parameter('arucos_yaml').get_parameter_value().string_value
        if yaml_param:
            self.arucos_path = Path(yaml_param)
        else:
            self.arucos_path = _find_maps_dir() / 'arucos.yaml'

        self.static_tf = StaticTransformBroadcaster(self)
        self.published = False

        # Intentar publicar inmediatamente, y reintentar cada 2s si no existe aun
        self.timer = self.create_timer(2.0, self._tick)
        self._tick()

    def _tick(self):
        if self.published:
            return

        if not self.arucos_path.exists():
            self.get_logger().info(
                f'Esperando {self.arucos_path} ...',
                throttle_duration_sec=5.0)
            return

        try:
            with open(self.arucos_path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f'Error leyendo {self.arucos_path}: {e}')
            return

        if not data:
            self.get_logger().warn(f'{self.arucos_path} esta vacio.')
            return

        transforms = []
        for name, info in data.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = f'{name}_aruco'
            t.transform.translation.x = float(info['world_x'])
            t.transform.translation.y = float(info['world_y'])
            t.transform.translation.z = 0.0

            yaw = float(info.get('yaw_world', 0.0))
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = float(math.sin(yaw * 0.5))
            t.transform.rotation.w = float(math.cos(yaw * 0.5))
            transforms.append(t)

            self.get_logger().info(
                f'TF publicada: {t.child_frame_id} -> '
                f'({info["world_x"]:.3f}, {info["world_y"]:.3f})')

        self.static_tf.sendTransform(transforms)
        self.published = True
        self.get_logger().info(
            f'Publicadas {len(transforms)} TFs estaticas de ArUcos.')
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PublicadorTfsArucos()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
