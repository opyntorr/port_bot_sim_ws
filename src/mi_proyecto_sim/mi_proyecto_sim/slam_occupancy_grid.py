#!/usr/bin/env python3
"""
SLAM Occupancy Grid con Fusi  n Dron + LiDAR.

Pipeline:
  1. Carga el PGM del dron como prior (mapa inicial)
  2. Actualiza celdas en tiempo real con LiDAR (Bresenham ray-tracing)
  3. Publica OccupancyGrid fusionado en /map
  4. Detecta conflictos con la ruta actual y solicita replanificaci  n
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
import numpy as np
import math
import os
import cv2


class SlamOccupancyGrid(Node):
    def __init__(self):
        super().__init__('slam_occupancy_grid')

        #        Par  metros                                                                                                                                           
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width', 120)
        self.declare_parameter('map_height', 120)
        self.declare_parameter('map_origin_x', -1.0)
        self.declare_parameter('map_origin_y', -5.0)
        self.declare_parameter('log_odds_free', -0.4)
        self.declare_parameter('log_odds_occupied', 0.9)
        self.declare_parameter('log_odds_max', 5.0)
        self.declare_parameter('log_odds_min', -5.0)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('replan_threshold', 5)
        self.declare_parameter('pgm_path', '')
        self.declare_parameter('pgm_resolution', 0.01)
        self.declare_parameter('pgm_origin_x', 0.0)
        self.declare_parameter('pgm_origin_y', -4.4)
        # Rotaci  n manual map   odom (radianes). ArUco no detecta yaw bien desde arriba.
        # Probar 1.5708 (90  ), -1.5708 (-90  ), 3.1416 (180  ) seg  n orientaci  n Gazebo.
        self.declare_parameter('map_to_odom_yaw', 1.5708)
        # Si False, no publica la TF estatica map->odom (util cuando un nodo
        # externo como AMCL se encarga de la localizacion).
        self.declare_parameter('publish_map_to_odom_tf', True)

        self.resolution = self.get_parameter('map_resolution').value
        self.width = self.get_parameter('map_width').value
        self.height = self.get_parameter('map_height').value
        self.origin_x = self.get_parameter('map_origin_x').value
        self.origin_y = self.get_parameter('map_origin_y').value
        self.lo_free = self.get_parameter('log_odds_free').value
        self.lo_occ = self.get_parameter('log_odds_occupied').value
        self.lo_max = self.get_parameter('log_odds_max').value
        self.lo_min = self.get_parameter('log_odds_min').value
        pub_rate = self.get_parameter('publish_rate').value
        self.replan_thresh = self.get_parameter('replan_threshold').value
        pgm_path = self.get_parameter('pgm_path').value
        self.pgm_res = self.get_parameter('pgm_resolution').value
        self.pgm_ox = self.get_parameter('pgm_origin_x').value
        self.pgm_oy = self.get_parameter('pgm_origin_y').value
        self.manual_yaw = self.get_parameter('map_to_odom_yaw').value
        self.publish_tf = self.get_parameter('publish_map_to_odom_tf').value

        #        Grid de log-odds (inicialmente unknown = 0.0)                         
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float64)

        # Cargar PGM del dron como prior
        if pgm_path and os.path.exists(pgm_path):
            import yaml
            yaml_path = pgm_path.replace('.pgm', '.yaml')
            if os.path.exists(yaml_path):
                try:
                    with open(yaml_path, 'r') as f:
                        map_data = yaml.safe_load(f)
                        self.pgm_res = map_data.get('resolution', self.pgm_res)
                        origin = map_data.get('origin', [self.pgm_ox, self.pgm_oy, 0.0])
                        self.pgm_ox = origin[0]
                        self.pgm_oy = origin[1]
                        self.get_logger().info(f"YAML Cargado! Origen PGM ajustado a Y={self.pgm_oy}")
                except Exception as e:
                    self.get_logger().error(f"Error cargando YAML: {e}")
            self._load_pgm_prior(pgm_path)
        else:
            self.get_logger().info('Sin PGM previo. Mapa inicia vacío (exploración pura).')

        #        Suscriptores                                                                                                                            
        qos_scan = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_filtered', self._scan_cb, qos_scan)

        # Odometr  a directa (el bridge no publica TF odom   base_footprint)
        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, qos_odom)
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        #        TF2 (para leer carrito_aruco y publicar map   odom)             
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Offset map     odom (se calcula al recibir ArUco + primer odom)
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_yaw = 0.0
        self.offset_calibrated = False

        qos_path = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_sub = self.create_subscription(
            Path, '/rrt_path', self._path_cb, qos_path)
        # ── Publicadores ─────────────────────────────────────────
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_map)
        self.replan_pub = self.create_publisher(Empty, '/replan', 10)

        # ── Estado interno ───────────────────────────────────────
        self.lidar_enabled = False      # LiDAR bloqueado hasta recibir la primera ruta
        self.current_path = []          # Lista de (x_m, y_m) del path actual
        self.map_dirty = True           # Forzar publicaci  n inicial
        self.last_path_cells = set()    # Celdas ocupadas sobre la ruta (cache)
        self.cells_since_pub = 0        # Celdas cambiadas desde   ltima publicaci  n
        self.min_cells_to_publish = 50  # M  nimo de cambios para republicar

        #        Contadores de diagn  stico                                                                                     
        self.scan_count = 0
        self.tf_fail_count = 0
        self.tf_ok_count = 0
        self.cells_updated = 0
        self.last_tf_error = ''

        #        Timer de publicaci  n                                                                                                    
        period = 1.0 / pub_rate
        self.pub_timer = self.create_timer(period, self._publish_map)

        #        Timer de diagn  stico (cada 3 segundos)                                              
        self.diag_timer = self.create_timer(3.0, self._diag_log)

        self.get_logger().info(
            f'SLAM OccGrid iniciado: {self.width}x{self.height} @ {self.resolution} m/cel, '
            f'origen ({self.origin_x}, {self.origin_y}). LiDAR en espera de ruta...')

    #                                                                                                                                                                                           
    #  Carga del PGM del dron como prior
    #                                                                                                                                                                                           
    def _load_pgm_prior(self, path):
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f'No se pudo leer PGM: {path}')
            return

        # Voltear verticalmente (OpenCV top-down     ROS bottom-up)
        img = cv2.flip(img, 0)

        h_pgm, w_pgm = img.shape
        self.get_logger().info(f'PGM cargado: {w_pgm}x{h_pgm} px @ {self.pgm_res} m/px')

        # Redimensionar al resolution del SLAM
        scale = self.pgm_res / self.resolution
        new_w = max(1, int(w_pgm * scale))
        new_h = max(1, int(h_pgm * scale))
        img_resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)

        # Calcular offset:   en qu   celda del SLAM cae el origen del PGM?
        col_off = int((self.pgm_ox - self.origin_x) / self.resolution)
        row_off = int((self.pgm_oy - self.origin_y) / self.resolution)

        # Copiar pixels a log-odds dentro del rango v  lido
        loaded = 0
        for r in range(new_h):
            for c in range(new_w):
                gr = row_off + r
                gc = col_off + c
                if 0 <= gr < self.height and 0 <= gc < self.width:
                    pix = img_resized[r, c]
                    if pix < 50:
                        self.log_odds[gr, gc] = 2.0   # Ocupado
                    elif pix > 200:
                        self.log_odds[gr, gc] = -2.0  # Libre
                    # else: se queda en 0 (unknown)
                    loaded += 1

        self.get_logger().info(
            f'Prior del dron cargado: {loaded} celdas inicializadas '
            f'(offset col={col_off}, row={row_off})')

    #                                                                                                                                                                                           
    #  Conversi  n mundo     grid
    #                                                                                                                                                                                           
    def _world_to_grid(self, wx, wy):
        col = int((wx - self.origin_x) / self.resolution)
        row = int((wy - self.origin_y) / self.resolution)
        return col, row

    def _in_grid(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    #                                                                                                                                                                                           
    #  Bresenham line algorithm
    #                                                                                                                                                                                           
    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        """Devuelve lista de (col, row) desde (x0,y0) hasta (x1,y1) exclusive del   ltimo."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return cells

    #                                                                                                                                                                                           
    #  Callback de odometr  a
    #                                                                                                                                                                                           
    def _odom_cb(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Calibrar offset map   odom usando ArUco (una sola vez)
        if not self.offset_calibrated:
            try:
                tf = self.tf_buffer.lookup_transform('map', 'carrito_aruco',
                                                      rclpy.time.Time())
                aruco_x = tf.transform.translation.x
                aruco_y = tf.transform.translation.y

                # Usar yaw manual (ArUco no detecta yaw bien desde arriba)
                self.offset_yaw = self.manual_yaw

                # Para calcular la traslaci  n, necesitamos:
                # aruco_pos = offset_translation + R(offset_yaw) * odom_pos
                # offset_translation = aruco_pos - R(offset_yaw) * odom_pos
                cos_y = math.cos(self.offset_yaw)
                sin_y = math.sin(self.offset_yaw)
                rotated_odom_x = odom_x * cos_y - odom_y * sin_y
                rotated_odom_y = odom_x * sin_y + odom_y * cos_y

                self.offset_x = aruco_x - rotated_odom_x
                self.offset_y = aruco_y - rotated_odom_y
                self.offset_calibrated = True

                self.get_logger().info(
                    f'    Offset map   odom CALIBRADO: dx={self.offset_x:.3f}, '
                    f'dy={self.offset_y:.3f}, dyaw={math.degrees(self.offset_yaw):.1f}  ')

                # Publicar TF est  tica map   odom (omitir si un nodo externo
                # como AMCL se encarga de la localizacion)
                if self.publish_tf:
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'map'
                    t.child_frame_id = 'odom'
                    t.transform.translation.x = self.offset_x
                    t.transform.translation.y = self.offset_y
                    t.transform.translation.z = 0.0
                    t.transform.rotation.z = math.sin(self.offset_yaw / 2.0)
                    t.transform.rotation.w = math.cos(self.offset_yaw / 2.0)
                    self.tf_broadcaster.sendTransform(t)

            except Exception:
                pass  # ArUco a  n no disponible, se reintenta en el pr  ximo odom

        # Aplicar rotaci  n + traslaci  n para obtener posici  n en frame map
        cos_y = math.cos(self.offset_yaw)
        sin_y = math.sin(self.offset_yaw)
        self.robot_x = self.offset_x + odom_x * cos_y - odom_y * sin_y
        self.robot_y = self.offset_y + odom_x * sin_y + odom_y * cos_y
        self.robot_yaw = odom_yaw + self.offset_yaw

    #                                                                                                                                                                                           
    #  Callback del LiDAR filtrado
    #                                                                                                                                                                                           
    def _scan_cb(self, msg):
        self.scan_count += 1

        # Ignorar LiDAR si no est activado aun
        if not self.lidar_enabled:
            return

        # Pose del robot en frame map: leida de TF (publicada por AMCL).
        # Asi los rayos se trazan usando la pose CORREGIDA, no la odom cruda.
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception as e:
            self.tf_fail_count += 1
            self.last_tf_error = f'TF map->base_footprint no disponible: {e}'
            return

        self.tf_ok_count += 1
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        robot_col, robot_row = self._world_to_grid(rx, ry)
        updated = 0

        for i, r in enumerate(msg.ranges):
            # Filtrar rangos inv  lidos
            if r < msg.range_min or r > msg.range_max or math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment + yaw

            # Punto de impacto en mundo
            hit_x = rx + r * math.cos(angle)
            hit_y = ry + r * math.sin(angle)
            hit_col, hit_row = self._world_to_grid(hit_x, hit_y)

            # Trazar rayo con Bresenham
            cells = self._bresenham(robot_col, robot_row, hit_col, hit_row)

            # Celdas intermedias     libre
            for c, rr in cells[:-1]:
                if self._in_grid(c, rr):
                    self.log_odds[rr, c] = max(self.lo_min,
                                               self.log_odds[rr, c] + self.lo_free)
                    updated += 1

            # Celda final     ocupada
            if cells:
                ec, er = cells[-1]
                if self._in_grid(ec, er):
                    self.log_odds[er, ec] = min(self.lo_max,
                                                self.log_odds[er, ec] + self.lo_occ)
                    updated += 1

        self.cells_updated += updated
        self.cells_since_pub += updated
        if self.cells_since_pub >= self.min_cells_to_publish:
            self.map_dirty = True

    #                                                                                                                                                                                           
    #  Diagn  stico peri  dico
    #                                                                                                                                                                                           
    def _diag_log(self):
        status = "ACTIVO" if self.lidar_enabled else "ESPERANDO RUTA"
        self.get_logger().info(
            f'[DIAG] LiDAR: {status} | Scans: {self.scan_count} | '
            f'TF OK: {self.tf_ok_count} | TF FAIL: {self.tf_fail_count} | '
            f'Celdas actualizadas: {self.cells_updated}')
        if self.tf_fail_count > 0 and self.tf_ok_count == 0:
            self.get_logger().warn(f'[DIAG]   ltimo error: {self.last_tf_error}')

    #                                                                                                                                                                                           
    #  Callback de la ruta RRT
    #                                                                                                                                                                                           
    def _path_cb(self, msg):
        self.current_path = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses]
        
        # ACTIVACI  N DEL LIDAR: Solo al recibir la primera ruta
        if not self.lidar_enabled:
            self.lidar_enabled = True
            self.get_logger().info('       Primera ruta recibida! Activando LiDAR para mapeo din  mico.')

        self.get_logger().info(
            f'Ruta recibida: {len(self.current_path)} waypoints')

    #                                                                                                                                                                                           
    #  Publicaci  n peri  dica del mapa y detecci  n de conflictos
    #                                                                                                                                                                                           
    def _publish_map(self):
        if not self.map_dirty:
            return

        self.cells_since_pub = 0  # Resetear contador

        #        Construir OccupancyGrid                                                                                           
        og = OccupancyGrid()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'map'
        og.info.resolution = float(self.resolution)
        og.info.width = self.width
        og.info.height = self.height
        og.info.origin = Pose()
        og.info.origin.position.x = float(self.origin_x)
        og.info.origin.position.y = float(self.origin_y)
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0

        # Convertir log-odds     probabilidad     int8
        prob = 1.0 / (1.0 + np.exp(-self.log_odds))  # 0   1
        data = np.full(self.log_odds.shape, -1, dtype=np.int8)
        # Celdas con informaci  n (log-odds != 0     han sido observadas)
        observed = np.abs(self.log_odds) > 0.01
        data[observed] = (prob[observed] * 100).astype(np.int8)
        og.data = data.flatten().tolist()

        self.map_pub.publish(og)
        self.map_dirty = False

        #        Detecci  n de conflictos con la ruta                                                       
        self._check_path_conflicts()

    def _check_path_conflicts(self):
        if not self.current_path:
            return

        conflicts = 0
        for wx, wy in self.current_path:
            c, r = self._world_to_grid(wx, wy)
            if self._in_grid(c, r) and self.log_odds[r, c] > 1.0:
                conflicts += 1

        if conflicts >= self.replan_thresh:
            self.get_logger().warn(
                f'  {conflicts} celdas de la ruta est  n ahora OCUPADAS! '
                f'Solicitando replanificaci  n...')
            self.replan_pub.publish(Empty())
            # Limpiar path para no re-triggerear inmediatamente
            self.current_path = []


def main(args=None):
    rclpy.init(args=args)
    node = SlamOccupancyGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
