#!/usr/bin/env python3
"""
Nodo que lee arucos.yaml (generado por mision_dron) y publica TFs estaticas
en frame `map` (el de slam_toolbox).

El stitching del dron entrega cada ArUco en coordenadas absolutas del mundo
(world_x, world_y, yaw_world). Como slam_toolbox ancla su frame `map` a la
pose del carrito al primer scan, transformamos cada ArUco para que queden
relativos al carrito (carrito_aruco -> (0,0,0); meta_aruco -> pose relativa).

Tambien publica una TF estatica `map -> map_dron_origin` que reposiciona el
mapa stitched para que coincida con el frame de SLAM en RViz.
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

        # Inicializar TF2 Buffer y Listener aqui en __init__ para evitar
        # problemas de concurrencia al crear subscripciones durante el spin
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

        try:
            trans_car = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            
            # SLAM esta activo, calcular alineacion perfecta
            mx = trans_car.transform.translation.x
            my = trans_car.transform.translation.y
            mq = trans_car.transform.rotation
            import math
            myaw = math.atan2(2.0*(mq.w*mq.z + mq.x*mq.y), 1.0 - 2.0*(mq.y*mq.y + mq.z*mq.z))

            carrito = data['carrito']
            cx = float(carrito['world_x'])
            cy = float(carrito['world_y'])
            ctheta = float(carrito.get('yaw_world', 0.0))

            phi = myaw - ctheta
            tx = mx - (cx * math.cos(phi) - cy * math.sin(phi))
            ty = my - (cx * math.sin(phi) + cy * math.cos(phi))
            
            self.get_logger().info(f'Alineacion OptiTrack->SLAM completada. (tx={tx:.3f}, ty={ty:.3f})')
            aligned = True
            
        except Exception as e:
            self.get_logger().info('Esperando SLAM... Publicando mapa stitching sin alinear temporalmente.', throttle_duration_sec=2.0)
            # SLAM aun no arranca, publicar centrado en 0 temporalmente
            tx, ty, phi = 0.0, 0.0, 0.0
            aligned = False

        transforms = []
        stamp = self.get_clock().now().to_msg()

        # 1. TF map -> map_dron_origin
        t_origin = TransformStamped()
        t_origin.header.stamp = stamp
        t_origin.header.frame_id = 'map'
        t_origin.child_frame_id = 'map_dron_origin'
        t_origin.transform.translation.x = float(tx)
        t_origin.transform.translation.y = float(ty)
        t_origin.transform.translation.z = 0.0
        import math
        t_origin.transform.rotation.x = 0.0
        t_origin.transform.rotation.y = 0.0
        t_origin.transform.rotation.z = float(math.sin(phi * 0.5))
        t_origin.transform.rotation.w = float(math.cos(phi * 0.5))
        transforms.append(t_origin)

        # 2. ArUcos relativos a map_dron_origin (su frame natural de OptiTrack/Stitching)
        for name, info in data.items():
            wx = float(info['world_x'])
            wy = float(info['world_y'])
            wyaw = float(info.get('yaw_world', 0.0))

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = 'map_dron_origin'
            t.child_frame_id = f'{name}_aruco'
            t.transform.translation.x = wx
            t.transform.translation.y = wy
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = float(math.sin(wyaw * 0.5))
            t.transform.rotation.w = float(math.cos(wyaw * 0.5))
            transforms.append(t)

        self.static_tf.sendTransform(transforms)
        
        # Solo detenemos el ciclo si ya logramos la alineacion definitiva
        if aligned:
            self.published = True
            self.timer.cancel()
            self.get_logger().info(f'Alineacion final anclada exitosamente.')


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
