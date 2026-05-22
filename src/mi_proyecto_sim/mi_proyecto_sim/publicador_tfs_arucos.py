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
        # carto_frozen_prior=True: el stack usa Cartographer con prior congelado
        # del PGM del dron, asi que el frame `map` YA es el frame del stitching.
        # No hay que alinear nada: publicamos `map -> map_dron_origin` como
        # identidad (compat con planner) y los ArUcos directo en sus coords del
        # YAML. Sin loop de espera ni validacion de displacement.
        self.declare_parameter('carto_frozen_prior', False)

        yaml_param = self.get_parameter('arucos_yaml').get_parameter_value().string_value
        if yaml_param:
            self.arucos_path = Path(yaml_param)
        else:
            self.arucos_path = _find_maps_dir() / 'arucos.yaml'

        self.carto_frozen_prior = bool(
            self.get_parameter('carto_frozen_prior').value)

        self.static_tf = StaticTransformBroadcaster(self)
        self.published = False

        # Inicializar TF2 Buffer y Listener aqui en __init__ para evitar
        # problemas de concurrencia al crear subscripciones durante el spin
        from tf2_ros import Buffer, TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Reintentar cada 100ms para capturar la alineacion lo antes posible
        # (antes de que el carrito se mueva de su posicion inicial)
        self.max_init_displacement = 0.15  # metros: tolerancia entre pose SLAM y origen
        self.timer = self.create_timer(0.1, self._tick)
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

        # Stack Cartographer con prior congelado: `map` ES el frame del stitching.
        # No alineamos nada; solo publicamos identity + arucos en sus coords del YAML.
        if self.carto_frozen_prior:
            self._publish_identity_chain(data)
            return

        try:
            trans_car = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())

            # SLAM esta activo, calcular alineacion perfecta
            mx = trans_car.transform.translation.x
            my = trans_car.transform.translation.y
            mq = trans_car.transform.rotation
            import math
            myaw = math.atan2(2.0*(mq.w*mq.z + mq.x*mq.y), 1.0 - 2.0*(mq.y*mq.y + mq.z*mq.z))

            # VALIDACION: SLAM ancla 'map' al primer scan, por lo que el carrito
            # debe estar muy cerca de (0,0) al inicio. Si ya se movio, la alineacion
            # quedara descalibrada -> abortar y reintentar en el siguiente tick.
            displacement = math.hypot(mx, my)
            if displacement > self.max_init_displacement:
                self.get_logger().warn(
                    f'Carrito a {displacement:.2f}m del origen SLAM '
                    f'(>{self.max_init_displacement}m). Esperando que se quede quieto antes de alinear...',
                    throttle_duration_sec=2.0)
                return

            carrito = data['carrito']
            cx = float(carrito['world_x'])
            cy = float(carrito['world_y'])
            ctheta = float(carrito.get('yaw_world', 0.0))

            phi = myaw - ctheta
            tx = mx - (cx * math.cos(phi) - cy * math.sin(phi))
            ty = my - (cx * math.sin(phi) + cy * math.cos(phi))

            self.get_logger().info(
                f'Alineacion OptiTrack->SLAM completada. '
                f'(tx={tx:.3f}, ty={ty:.3f}, despl_inicial={displacement:.3f}m)')
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

        # 2. ArUcos: la meta se ancla directamente en `map` (frame absoluto de SLAM)
        #    para desacoplarla de map_dron_origin y que no se mueva si SLAM hace
        #    loop closure o si la alineacion se recomputa. El resto sigue colgando
        #    de map_dron_origin como antes.
        for name, info in data.items():
            wx = float(info['world_x'])
            wy = float(info['world_y'])
            wyaw = float(info.get('yaw_world', 0.0))

            t = TransformStamped()
            t.header.stamp = stamp
            t.child_frame_id = f'{name}_aruco'

            if name == 'meta' and aligned:
                # Convertir (wx, wy, wyaw) de map_dron_origin a map: aplicar (tx, ty, phi)
                mx_meta = tx + (wx * math.cos(phi) - wy * math.sin(phi))
                my_meta = ty + (wx * math.sin(phi) + wy * math.cos(phi))
                myaw_meta = phi + wyaw
                t.header.frame_id = 'map'
                t.transform.translation.x = float(mx_meta)
                t.transform.translation.y = float(my_meta)
                t.transform.translation.z = 0.0
                t.transform.rotation.z = float(math.sin(myaw_meta * 0.5))
                t.transform.rotation.w = float(math.cos(myaw_meta * 0.5))
                self.get_logger().info(
                    f'Meta anclada en frame `map` en ({mx_meta:.3f}, {my_meta:.3f}, '
                    f'{math.degrees(myaw_meta):.1f} deg). No se vuelve a republicar.')
            else:
                t.header.frame_id = 'map_dron_origin'
                t.transform.translation.x = wx
                t.transform.translation.y = wy
                t.transform.translation.z = 0.0
                t.transform.rotation.z = float(math.sin(wyaw * 0.5))
                t.transform.rotation.w = float(math.cos(wyaw * 0.5))

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            transforms.append(t)

        self.static_tf.sendTransform(transforms)

        # Solo detenemos el ciclo si ya logramos la alineacion definitiva
        if aligned:
            self.published = True
            self.timer.cancel()
            self.get_logger().info(f'Alineacion final anclada exitosamente.')

    def _publish_identity_chain(self, data):
        """Stack Cartographer+prior congelado: `map` es el frame del stitching, no
        hay alineacion. Publica `map -> map_dron_origin` como identidad y los
        ArUcos directo en sus coords del YAML."""
        import math
        stamp = self.get_clock().now().to_msg()
        transforms = []

        t_origin = TransformStamped()
        t_origin.header.stamp = stamp
        t_origin.header.frame_id = 'map'
        t_origin.child_frame_id = 'map_dron_origin'
        t_origin.transform.rotation.w = 1.0
        transforms.append(t_origin)

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
            t.transform.rotation.z = float(math.sin(wyaw * 0.5))
            t.transform.rotation.w = float(math.cos(wyaw * 0.5))
            transforms.append(t)

        self.static_tf.sendTransform(transforms)
        self.published = True
        self.timer.cancel()
        self.get_logger().info(
            f'Modo Cartographer+prior: publicadas {len(transforms)} TFs estaticas '
            f'(identity map->map_dron_origin + {len(data)} arucos).')


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
