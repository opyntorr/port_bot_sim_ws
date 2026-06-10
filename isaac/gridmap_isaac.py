#!/usr/bin/env python3
"""
TAREA 2 en Isaac Sim — Mapa de ocupacion probabilistico con el RTX lidar.

Identico en algoritmo a la version de Gazebo (log-odds + modelo inverso por
haz con Bresenham); el sensor es el RTX lidar 2D de scene_gridmap.py (/scan).
El nodo:
  * suscribe /scan (sensor_msgs/LaserScan) del RTX lidar,
  * obtiene la pose del lidar en 'odom' via TF (odom->base_footprint de la escena
    + base_footprint->lidar_frame de robot_state_publisher),
  * actualiza una rejilla en LOG-ODDS, publica /mapa_probabilistico (RViz),
  * autoguarda isaac_10_mapa_ocupacion.png cada ~10 s.

Correr (host, NO docker):
    source /opt/ros/humble/setup.bash
    source isaac/isaac_env.sh
    python3 isaac/gridmap_isaac.py     # con scene_gridmap.py + robot_state_publisher arriba
"""

import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sensor_models as sm

HERE = os.path.dirname(os.path.abspath(__file__))


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def bresenham(i0, j0, i1, j1):
    cells = []
    di, dj = abs(i1 - i0), abs(j1 - j0)
    si = 1 if i0 < i1 else -1
    sj = 1 if j0 < j1 else -1
    err = di - dj
    i, j = i0, j0
    while True:
        cells.append((i, j))
        if i == i1 and j == j1:
            break
        e2 = 2 * err
        if e2 > -dj:
            err -= dj
            i += si
        if e2 < di:
            err += di
            j += sj
    return cells


class GridMapIsaac(Node):
    def __init__(self):
        super().__init__('gridmap_isaac',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        gp = lambda n, v: self.declare_parameter(n, v).value

        self.fixed_frame = gp('fixed_frame', 'odom')
        self.res = gp('resolution', 0.05)
        size_m = gp('size_m', 12.0)
        self.origin_x = gp('origin_x', -size_m / 2.0)
        self.origin_y = gp('origin_y', -size_m / 2.0)
        self.nx = int(size_m / self.res)
        self.ny = int(size_m / self.res)
        self.max_range = gp('max_range', 5.0)                   # como Gazebo gz_10
        self.target_beams = gp('target_beams', 400)            # submuestreo del /scan RTX
        self.clamp = gp('clamp', 6.0)
        self.inject_ms200 = gp('inject_ms200', False)          # el RTX lidar ya trae ruido
        self.out_dir = gp('output_dir', os.path.join(HERE, 'figs'))
        self.rng = np.random.default_rng(0)

        p_occ, p_free = gp('p_occ', 0.7), gp('p_free', 0.4)
        self.l_occ = math.log(p_occ / (1 - p_occ))
        self.l_free = math.log(p_free / (1 - p_free))

        self.L = np.zeros((self.ny, self.nx))
        self.path = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(OccupancyGrid, '/mapa_probabilistico', 1)
        self.create_timer(0.5, self.publish_map)
        self.n_scans = 0
        self.create_timer(10.0, self.save_plot)
        self.get_logger().info('gridmap_isaac listo (esperando /scan y TF odom->lidar_frame)...')

    def w2g(self, x, y):
        j = int((x - self.origin_x) / self.res)
        i = int((y - self.origin_y) / self.res)
        return i, j

    def in_bounds(self, i, j):
        return 0 <= i < self.ny and 0 <= j < self.nx

    def scan_cb(self, scan: LaserScan):
        # TF en el INSTANTE del barrido (no el "ultimo"): al orbitar el robot rota,
        # y usar la TF mas reciente metia un giro/blur por el desfase. Con el stamp
        # del scan cada barrido se coloca con la pose real de captura.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, scan.header.frame_id,
                rclpy.time.Time.from_msg(scan.header.stamp))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            try:   # respaldo: ultima disponible (raro; solo si el stamp no esta en el buffer)
                tf = self.tf_buffer.lookup_transform(
                    self.fixed_frame, scan.header.frame_id, rclpy.time.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tyaw = yaw_from_quat(tf.transform.rotation)
        self.path.append((tx, ty))

        i0, j0 = self.w2g(tx, ty)
        if not self.in_bounds(i0, j0):
            return

        rmax = min(self.max_range, scan.range_max)
        # Submuestreo a ~target_beams haces: el RTX lidar da 3200 pts/360deg y el
        # Bresenham en Python no alcanza los 55 Hz -> la cola atrasa la pose y el
        # mapa sale girado. Con ~400 haces (resolucion tipo MS200 real) el callback
        # sigue el ritmo y la pose (ultima TF) coincide con el barrido.
        step = max(1, len(scan.ranges) // self.target_beams)
        for idx in range(0, len(scan.ranges), step):
            r = scan.ranges[idx]
            a = scan.angle_min + idx * scan.angle_increment
            if self.inject_ms200 and math.isfinite(r) and r < rmax:
                r = sm.corrupt_lidar_range(r, self.rng, rmax)
            hit = math.isfinite(r) and scan.range_min <= r < rmax
            if not hit:
                continue
            end_r = r
            ex = tx + end_r * math.cos(tyaw + a)
            ey = ty + end_r * math.sin(tyaw + a)
            i1, j1 = self.w2g(ex, ey)
            i1 = min(max(i1, 0), self.ny - 1)
            j1 = min(max(j1, 0), self.nx - 1)
            cells = bresenham(i0, j0, i1, j1)
            for (ci, cj) in cells[:-1]:
                self.L[ci, cj] += self.l_free
            ci, cj = cells[-1]
            self.L[ci, cj] += self.l_occ if hit else self.l_free
        np.clip(self.L, -self.clamp, self.clamp, out=self.L)
        self.n_scans += 1

    def prob(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.L))

    def publish_map(self):
        if self.n_scans == 0:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.fixed_frame
        msg.info.resolution = self.res
        msg.info.width = self.nx
        msg.info.height = self.ny
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        data = np.full(self.L.shape, -1, dtype=np.int8)
        seen = self.L != 0.0
        prob = self.prob()
        data[seen] = (prob[seen] * 100).astype(np.int8)
        msg.data = data.flatten().tolist()
        self.pub.publish(msg)

    def save_plot(self):
        if self.n_scans == 0:
            return
        os.makedirs(self.out_dir, exist_ok=True)
        extent = [self.origin_x, self.origin_x + self.nx * self.res,
                  self.origin_y, self.origin_y + self.ny * self.res]
        plt.figure(figsize=(8, 8))
        plt.imshow(self.prob(), origin='lower', extent=extent, cmap='Greys', vmin=0, vmax=1)
        if self.path:
            p = np.array(self.path)
            plt.plot(p[:, 0], p[:, 1], 'r-', lw=1, label='trayectoria lidar')
            plt.legend()
        prob = self.prob()
        occ = np.argwhere(prob >= 0.6)
        if occ.size:
            xs = self.origin_x + occ[:, 1] * self.res
            ys = self.origin_y + occ[:, 0] * self.res
            if self.path:
                xs = np.concatenate([xs, p[:, 0]]); ys = np.concatenate([ys, p[:, 1]])
            m = 1.0
            plt.xlim(xs.min() - m, xs.max() + m)
            plt.ylim(ys.min() - m, ys.max() + m)
        plt.title(f'Isaac Sim: mapa de ocupacion ({self.n_scans} escaneos)\n'
                  'negro=ocupado, blanco=libre, gris=desconocido')
        plt.xlabel('x [m]'); plt.ylabel('y [m]')
        plt.tight_layout()
        plt.savefig(os.path.join(self.out_dir, 'isaac_10_mapa_ocupacion.png'), dpi=130)
        plt.close()
        self.get_logger().info(f'Mapa guardado ({self.n_scans} escaneos) en {self.out_dir}')


def main():
    rclpy.init()
    node = GridMapIsaac()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_plot()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
