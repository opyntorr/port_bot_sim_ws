#!/usr/bin/env python3
"""
Guarda el OccupancyGrid de /map a disco en formato Nav2 (PGM + YAML), desde la LAPTOP.

Se suscribe a /map (que llega por el bridge DDS), escribe el mapa en la carpeta local
(default ~/jetauto_maps) y ADEMAS lo sube al Orin (~/maps) por scp, para que
navegar_real.launch.py pueda servirlo. Asi se guarda en AMBOS lados con UN comando,
sin que el usuario tenga que hacer SSH.

Es una copia standalone (en jetauto_rviz) del guardar_mapa_slam.py de mi_proyecto_sim,
para no tener que compilar todo ese paquete en la laptop. Mismo formato de salida.

Uso (normalmente via guardar_mapa.launch.py):
    ros2 run jetauto_rviz guardar_mapa --ros-args -p map_name:=mi_mapa
Params:
    output_dir   (default ~/jetauto_maps)   carpeta local en la laptop
    map_name     (default mapa_<timestamp>) nombre sin extension
    map_topic    (default /map)
    orin_host    (default jetson@10.42.1.1)  destino scp ('' = no subir al Orin)
    orin_dir     (default /home/jetson/maps) carpeta destino en el Orin
"""

import os
import subprocess
from datetime import datetime

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapSaverLaptop(Node):
    def __init__(self):
        super().__init__('guardar_mapa_laptop')

        self.declare_parameter('output_dir', os.path.expanduser('~/jetauto_maps'))
        self.declare_parameter('map_name', '')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('occupied_thresh', 0.65)
        self.declare_parameter('free_thresh', 0.196)
        self.declare_parameter('orin_host', 'jetson@10.42.1.1')
        self.declare_parameter('orin_dir', '/home/jetson/maps')

        self.output_dir = self.get_parameter('output_dir').value or os.path.expanduser('~/jetauto_maps')
        name = self.get_parameter('map_name').value
        if not name:
            name = f'mapa_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
        self.map_name = name
        topic = self.get_parameter('map_topic').value
        self.occ_th = float(self.get_parameter('occupied_thresh').value)
        self.free_th = float(self.get_parameter('free_thresh').value)
        self.orin_host = self.get_parameter('orin_host').value
        self.orin_dir = self.get_parameter('orin_dir').value

        os.makedirs(self.output_dir, exist_ok=True)

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
            f'(guardara en {self.output_dir}/{self.map_name}.{{pgm,yaml}} '
            f'+ copia al Orin {self.orin_host}:{self.orin_dir})'
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
        yaw = np.arctan2(
            2.0 * (oq.w * oq.z + oq.x * oq.y),
            1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z),
        )

        data = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
        img = np.full((h, w), 205, dtype=np.uint8)
        img[data == 0] = 254
        img[(data >= 0) & (data <= int(self.free_th * 100))] = 254
        img[data >= int(self.occ_th * 100)] = 0
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

        self.get_logger().info(f'Mapa guardado (laptop): {pgm_path} ({w}x{h} @ {res:.4f} m/px)')
        self.get_logger().info(f'YAML (laptop):          {yaml_path}')

        self._copiar_al_orin(pgm_path, yaml_path)

        self.create_timer(0.1, lambda: rclpy.shutdown())

    def _copiar_al_orin(self, pgm_path, yaml_path):
        """Sube el .pgm y .yaml al Orin por scp (para que navegar_real lo sirva)."""
        if not self.orin_host:
            self.get_logger().info('orin_host vacio: no se sube al Orin.')
            return
        dest = f'{self.orin_host}:{self.orin_dir}/'
        try:
            subprocess.run(
                ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
                 '-o', 'StrictHostKeyChecking=no', self.orin_host,
                 f'mkdir -p {self.orin_dir}'],
                check=True, timeout=20,
            )
            subprocess.run(
                ['scp', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
                 '-o', 'StrictHostKeyChecking=no', pgm_path, yaml_path, dest],
                check=True, timeout=40,
            )
            self.get_logger().info(
                f'Copia subida al Orin: {self.orin_dir}/{self.map_name}.{{pgm,yaml}}')
        except Exception as e:
            self.get_logger().warn(
                f'No se pudo subir al Orin ({e}). El mapa local SI quedo guardado. '
                f'Subelo a mano: scp {pgm_path} {yaml_path} {dest}')


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverLaptop()
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
