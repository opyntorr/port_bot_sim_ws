#!/usr/bin/env python3
"""
Guarda el OccupancyGrid actual de /map a disco en formato Nav2 (PGM + YAML).

Uso:
    # Default: guarda en <ws>/src/mi_proyecto_sim/maps/mapa_<YYYYMMDD_HHMMSS>.
    # Bajo Docker ese path cae en el volumen montado, asi el mapa queda
    # persistente en el host.
    ros2 run mi_proyecto_sim guardar_mapa_slam.py

    # Especificar carpeta y/o nombre:
    ros2 run mi_proyecto_sim guardar_mapa_slam.py --ros-args \
        -p output_dir:=/ros2_ws/src/mi_proyecto_sim/maps \
        -p map_name:=lab_v2

    # Otro topic (default /map):
    ros2 run mi_proyecto_sim guardar_mapa_slam.py --ros-args \
        -p map_topic:=/map_dron

El nodo termina solo despues de escribir el archivo.
"""

import os
from datetime import datetime

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _default_maps_dir():
    """src/mi_proyecto_sim/maps en la raiz del workspace.

    Bajo Docker (./src montado en /ros2_ws/src) esta ruta cae dentro del
    volumen montado, por lo que el mapa queda en el host automaticamente.
    """
    try:
        pkg_share = get_package_share_directory('mi_proyecto_sim')
        ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
        return os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'maps')
    except Exception:
        return os.getcwd()


class MapSaver(Node):
    def __init__(self):
        super().__init__('guardar_mapa_slam')

        self.declare_parameter('output_dir', '')
        self.declare_parameter('map_name', '')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('free_thresh', 0.196)

        self.output_dir = self.get_parameter('output_dir').value or _default_maps_dir()
        name = self.get_parameter('map_name').value
        if not name:
            name = f'mapa_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        self.map_name = name
        topic = self.get_parameter('map_topic').value
        self.occ_th = float(self.get_parameter('occupied_thresh').value)
        self.free_th = float(self.get_parameter('free_thresh').value)

        os.makedirs(self.output_dir, exist_ok=True)

        # /map usa QoS Transient Local: nos llega el ultimo mensaje al suscribirnos
        # aunque el publisher haya enviado solo una vez.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(OccupancyGrid, topic, self._cb, qos)
        self.saved = False
        self.get_logger().info(
            f'Esperando OccupancyGrid en {topic}... '
            f'(guardara en {self.output_dir}/{self.map_name}.{{pgm,yaml}})'
        )

    def _cb(self, msg: OccupancyGrid):
        if self.saved:
            return
        self.saved = True

        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        oq = msg.info.origin.orientation
        # Yaw del origin (rotacion alrededor de Z)
        yaw = np.arctan2(
            2.0 * (oq.w * oq.z + oq.x * oq.y),
            1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z),
        )

        # Mapeo OccupancyGrid -> escala de grises (convencion Nav2 map_saver):
        #   -1            -> 205 (gris, desconocido)
        #   >= occ*100    -> 0   (negro, ocupado)
        #   <= free*100   -> 254 (blanco, libre)
        #   resto         -> 205
        data = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[data == 0] = 254
        img[(data >= 0) & (data <= int(self.free_th * 100))] = 254
        img[data >= int(self.occ_th * 100)] = 0

        # OccupancyGrid se publica con origen en la esquina inferior izquierda
        # (row 0 = abajo). El formato PGM tiene row 0 = arriba. Hay que voltear.
        img = np.flipud(img)

        pgm_path = os.path.join(self.output_dir, f'{self.map_name}.pgm')
        yaml_path = os.path.join(self.output_dir, f'{self.map_name}.yaml')

        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
            f.write(img.tobytes())

        yaml_data = {
            'image': f'{self.map_name}.pgm',
            'mode': 'trinary',
            'resolution': float(res),
            'origin': [float(ox), float(oy), float(yaw)],
            'negate': 0,
            'occupied_thresh': float(self.occ_th),
            'free_thresh': float(self.free_th),
        }
        with open(yaml_path, 'w') as f:
            yaml.safe_dump(yaml_data, f, default_flow_style=None, sort_keys=False)

        self.get_logger().info(
            f'Mapa guardado: {pgm_path} ({w}x{h} @ {res:.4f} m/px)'
        )
        self.get_logger().info(f'YAML:           {yaml_path}')

        # Forzar shutdown limpio
        self.create_timer(0.1, lambda: rclpy.shutdown())


def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
