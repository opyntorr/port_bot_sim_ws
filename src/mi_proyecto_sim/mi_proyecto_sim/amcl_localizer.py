#!/usr/bin/env python3
"""
AMCL casero (Adaptive Monte Carlo Localization) implementado a mano.

Filtro de particulas para localizar el carrito en un mapa estatico conocido.
Publica TF map -> odom corrigiendo continuamente la deriva de la odometria
de ruedas.

Referencia: Thrun, Burgard, Fox - "Probabilistic Robotics" cap. 4-6, 8.

Componentes:
  - Motion model: odometry-based sample motion model (Tabla 5.6).
  - Sensor model: likelihood field (Tabla 6.3) usando distance transform
    del mapa para evaluar verosimilitud de los endpoints del scan.
  - Resampling: low-variance sampler (Tabla 4.4) cuando N_eff < N/2.
  - Estimacion: media ponderada para (x, y) y media circular para yaw.

Topics:
  /map           (sub) OccupancyGrid estatico (solo se usa el primero).
  /scan_filtered (sub) LaserScan.
  /odom          (sub) Odometry de ruedas.
  /initialpose   (sub) PoseWithCovarianceStamped (RViz "2D Pose Estimate").
  /amcl_pose     (pub) PoseWithCovarianceStamped (pose estimada).
  /particlecloud (pub) PoseArray (visualizacion).
  TF map->odom  (pub) Continuo.

Inicializacion: por defecto intenta leer la TF map->carrito_aruco (publicada
por publicador_tfs_arucos) para sembrar las particulas alrededor. Si no
existe, usa los parametros init_x/init_y/init_yaw.
"""

import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import (
    Pose, PoseArray, PoseWithCovarianceStamped, TransformStamped,
)
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class AmclLocalizer(Node):
    def __init__(self):
        super().__init__('amcl_localizer')

        # Parametros del filtro
        self.declare_parameter('num_particles', 500)
        # Motion model noise (Thrun alpha1-4)
        self.declare_parameter('alpha1', 0.2)
        self.declare_parameter('alpha2', 0.2)
        self.declare_parameter('alpha3', 0.2)
        self.declare_parameter('alpha4', 0.2)
        # Sensor model (likelihood field)
        self.declare_parameter('z_hit', 0.95)
        self.declare_parameter('z_rand', 0.05)
        self.declare_parameter('sigma_hit', 0.2)
        self.declare_parameter('laser_max_range', 5.0)
        self.declare_parameter('laser_subsample', 10)
        # Gating: solo actualizar tras suficiente desplazamiento
        self.declare_parameter('update_min_d', 0.10)
        self.declare_parameter('update_min_a', 0.10)
        # Inicializacion
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('init_std_xy', 0.3)
        self.declare_parameter('init_std_yaw', 0.3)
        self.declare_parameter('init_from_aruco', True)

        self.N = int(self.get_parameter('num_particles').value)
        self.a1 = self.get_parameter('alpha1').value
        self.a2 = self.get_parameter('alpha2').value
        self.a3 = self.get_parameter('alpha3').value
        self.a4 = self.get_parameter('alpha4').value
        self.z_hit = self.get_parameter('z_hit').value
        self.z_rand = self.get_parameter('z_rand').value
        self.sigma_hit = self.get_parameter('sigma_hit').value
        self.laser_max = self.get_parameter('laser_max_range').value
        self.laser_subsample = int(self.get_parameter('laser_subsample').value)
        self.update_min_d = self.get_parameter('update_min_d').value
        self.update_min_a = self.get_parameter('update_min_a').value
        self.init_from_aruco = self.get_parameter('init_from_aruco').value

        # Estado del filtro
        self.particles = None        # (N, 3): [x, y, yaw]
        self.weights = None          # (N,)
        self.map_array = None        # numpy (H, W)
        self.likelihood_field = None # (H, W) distancia en metros al obstaculo mas cercano
        self.map_res = None
        self.map_ox = None
        self.map_oy = None
        self.map_h = None
        self.map_w = None

        self.last_odom_xy_yaw = None     # ultima odom recibida
        self.last_update_xy_yaw = None   # ultima odom donde corrimos motion+sensor
        self.initialized = False
        self.estimated_pose = None       # (x, y, yaw) en frame map

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, qos_map)
        self.create_subscription(LaserScan, '/scan_filtered', self._scan_cb, qos_be)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_be)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self._initpose_cb, 10)

        self.pcloud_pub = self.create_publisher(PoseArray, '/particlecloud', 1)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publicar TF + visualizacion a 20 Hz
        self.pub_timer = self.create_timer(0.05, self._publish_state)

        self.get_logger().info(f'AMCL casero inicializado con {self.N} particulas.')

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def _map_cb(self, msg):
        # Solo aceptamos el primer mapa (es el "prior" estatico para localizar).
        if self.map_array is not None:
            return
        self.map_res = msg.info.resolution
        self.map_ox = msg.info.origin.position.x
        self.map_oy = msg.info.origin.position.y
        self.map_w = msg.info.width
        self.map_h = msg.info.height
        arr = np.array(msg.data, dtype=np.int8).reshape(self.map_h, self.map_w)
        self.map_array = arr

        # Likelihood field: distancia (m) de cada celda al obstaculo mas cercano.
        # cv2.distanceTransform mide distancia de pixeles != 0 al cero mas cercano,
        # asi que ponemos obstaculos=0 y libres=1.
        obstacles = (arr == 100).astype(np.uint8)
        if not obstacles.any():
            self.get_logger().warn('Mapa sin obstaculos: likelihood field uniforme.')
            self.likelihood_field = np.full(arr.shape, 1e3, dtype=np.float32)
        else:
            free = (1 - obstacles).astype(np.uint8)
            dist_px = cv2.distanceTransform(free, cv2.DIST_L2, 5)
            self.likelihood_field = (dist_px * self.map_res).astype(np.float32)

        self.get_logger().info(
            f'Mapa cargado: {self.map_w}x{self.map_h} @ {self.map_res:.3f} m/px. '
            f'Likelihood field listo.'
        )
        if not self.initialized:
            self._try_initialize()

    def _try_initialize(self):
        if self.likelihood_field is None:
            return
            
        x = self.get_parameter('init_x').value
        y = self.get_parameter('init_y').value
        yaw = self.get_parameter('init_yaw').value
        used_aruco = False
        
        if self.init_from_aruco:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'carrito_aruco', rclpy.time.Time())
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                yaw = yaw_from_quat(tf.transform.rotation)
                used_aruco = True
            except Exception:
                self.get_logger().info('Esperando la TF map -> carrito_aruco para inicializar AMCL...')
                return # Salir y reintentar despues (se llama en cada timer de publicacion)
                
        self._init_particles_around(x, y, yaw)
        self.initialized = True
        src = 'TF carrito_aruco' if used_aruco else 'parametros init_*'
        self.get_logger().info(
            f'Particulas inicializadas en ({x:.2f}, {y:.2f}, '
            f'{math.degrees(yaw):.1f} deg) desde {src}.'
        )

    def _init_particles_around(self, x, y, yaw):
        std_xy = self.get_parameter('init_std_xy').value
        std_yaw = self.get_parameter('init_std_yaw').value
        self.particles = np.zeros((self.N, 3), dtype=np.float64)
        self.particles[:, 0] = x + np.random.normal(0.0, std_xy, self.N)
        self.particles[:, 1] = y + np.random.normal(0.0, std_xy, self.N)
        self.particles[:, 2] = yaw + np.random.normal(0.0, std_yaw, self.N)
        self.weights = np.full(self.N, 1.0 / self.N)
        self.estimated_pose = (float(x), float(y), float(yaw))

    def _initpose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.get_logger().info(f'Pose inicial via /initialpose: ({x:.2f}, {y:.2f}).')
        self._init_particles_around(x, y, yaw)
        self.initialized = True

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.last_odom_xy_yaw = (x, y, yaw)
        if self.last_update_xy_yaw is None:
            self.last_update_xy_yaw = (x, y, yaw)

    def _scan_cb(self, msg):
        if not self.initialized or self.likelihood_field is None:
            return
        if self.last_odom_xy_yaw is None:
            return

        # Gating: solo procesar si nos movimos lo suficiente.
        xo, yo, to = self.last_odom_xy_yaw
        xl, yl, tl = self.last_update_xy_yaw
        d = math.hypot(xo - xl, yo - yl)
        da = abs(normalize_angle(to - tl))
        if d < self.update_min_d and da < self.update_min_a:
            return

        self._motion_update(self.last_update_xy_yaw, self.last_odom_xy_yaw)
        self._sensor_update(msg)
        n_eff = 1.0 / np.sum(self.weights ** 2)
        if n_eff < self.N / 2.0:
            self._resample()
        self._estimate_pose()
        self.last_update_xy_yaw = self.last_odom_xy_yaw

    # ------------------------------------------------------------
    # Filtro
    # ------------------------------------------------------------
    def _motion_update(self, odom_prev, odom_now):
        """Odometry-based sample motion model (Thrun, Tabla 5.6)."""
        x1, y1, t1 = odom_prev
        x2, y2, t2 = odom_now
        dx, dy = x2 - x1, y2 - y1
        trans = math.hypot(dx, dy)
        if trans < 0.01:
            # Giro en sitio: evitar atan2 ruidoso, todo el cambio se atribuye a rot2.
            rot1 = 0.0
            rot2 = normalize_angle(t2 - t1)
        else:
            rot1 = normalize_angle(math.atan2(dy, dx) - t1)
            rot2 = normalize_angle(t2 - t1 - rot1)

        sigma_r1 = math.sqrt(self.a1 * rot1 ** 2 + self.a2 * trans ** 2)
        sigma_tr = math.sqrt(self.a3 * trans ** 2 + self.a4 * (rot1 ** 2 + rot2 ** 2))
        sigma_r2 = math.sqrt(self.a1 * rot2 ** 2 + self.a2 * trans ** 2)

        rot1_hat = rot1 - np.random.normal(0.0, sigma_r1, self.N)
        trans_hat = trans - np.random.normal(0.0, sigma_tr, self.N)
        rot2_hat = rot2 - np.random.normal(0.0, sigma_r2, self.N)

        theta = self.particles[:, 2]
        self.particles[:, 0] += trans_hat * np.cos(theta + rot1_hat)
        self.particles[:, 1] += trans_hat * np.sin(theta + rot1_hat)
        self.particles[:, 2] = (theta + rot1_hat + rot2_hat + np.pi) % (2 * np.pi) - np.pi

    def _sensor_update(self, scan):
        """Likelihood field model (Thrun, Tabla 6.3), vectorizado en numpy."""
        ranges_full = np.array(scan.ranges, dtype=np.float32)
        n_rays = len(ranges_full)
        idx = np.arange(0, n_rays, self.laser_subsample)
        angles = scan.angle_min + idx.astype(np.float32) * scan.angle_increment
        ranges = ranges_full[idx]

        max_r = min(scan.range_max, self.laser_max)
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < max_r)
        if not valid.any():
            return
        angles = angles[valid]
        ranges = ranges[valid]

        # Endpoints en frame map para cada (particula, rayo): shape (N, M).
        cos_t = np.cos(self.particles[:, 2])[:, None]
        sin_t = np.sin(self.particles[:, 2])[:, None]
        cos_a = np.cos(angles)[None, :]
        sin_a = np.sin(angles)[None, :]
        ex = self.particles[:, 0:1] + ranges * (cos_t * cos_a - sin_t * sin_a)
        ey = self.particles[:, 1:2] + ranges * (sin_t * cos_a + cos_t * sin_a)

        cols = ((ex - self.map_ox) / self.map_res).astype(np.int32)
        rows = ((ey - self.map_oy) / self.map_res).astype(np.int32)
        in_map = (cols >= 0) & (cols < self.map_w) & (rows >= 0) & (rows < self.map_h)
        cols = np.clip(cols, 0, self.map_w - 1)
        rows = np.clip(rows, 0, self.map_h - 1)
        dists = self.likelihood_field[rows, cols].astype(np.float32)
        # Endpoints fuera del mapa: penalizar como si estuvieran muy lejos de obstaculo.
        dists[~in_map] = self.sigma_hit * 4.0

        p_hit = np.exp(-0.5 * (dists / self.sigma_hit) ** 2)
        p_rand = 1.0 / self.laser_max
        p = self.z_hit * p_hit + self.z_rand * p_rand
        # log-likelihood por particula (suma sobre rayos)
        log_w = np.sum(np.log(p + 1e-12), axis=1)
        log_w -= log_w.max()           # estabilidad numerica
        new_w = np.exp(log_w) * self.weights
        s = new_w.sum()
        if s > 0:
            self.weights = new_w / s
        else:
            # Colapso total: reiniciar pesos uniformes (no perdemos particulas).
            self.weights = np.full(self.N, 1.0 / self.N)

    def _resample(self):
        """Low-variance resampler (Thrun, Tabla 4.4)."""
        positions = (np.arange(self.N) + np.random.uniform()) / self.N
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0
        idx = np.searchsorted(cumulative, positions)
        idx = np.clip(idx, 0, self.N - 1)
        self.particles = self.particles[idx].copy()
        self.weights = np.full(self.N, 1.0 / self.N)

    def _estimate_pose(self):
        w = self.weights[:, None]
        mean_xy = np.sum(self.particles[:, :2] * w, axis=0)
        sin_mean = float(np.sum(np.sin(self.particles[:, 2]) * self.weights))
        cos_mean = float(np.sum(np.cos(self.particles[:, 2]) * self.weights))
        yaw = math.atan2(sin_mean, cos_mean)
        self.estimated_pose = (float(mean_xy[0]), float(mean_xy[1]), yaw)

    # ------------------------------------------------------------
    # Publicacion de TF + visualizacion
    # ------------------------------------------------------------
    def _publish_state(self):
        if not self.initialized:
            self._try_initialize()
            return
            
        if self.estimated_pose is None or self.last_odom_xy_yaw is None:
            return

        # TF map->odom: tal que map = T_map_odom * odom para la pose estimada.
        x_m, y_m, yaw_m = self.estimated_pose
        x_o, y_o, yaw_o = self.last_odom_xy_yaw
        dyaw = normalize_angle(yaw_m - yaw_o)
        c, s = math.cos(dyaw), math.sin(dyaw)
        tx = x_m - (c * x_o - s * y_o)
        ty = y_m - (s * x_o + c * y_o)

        stamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(dyaw / 2.0)
        t.transform.rotation.w = math.cos(dyaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Particle cloud (submuestreado para no saturar RViz)
        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = 'map'
        step = max(1, self.N // 200)
        for i in range(0, self.N, step):
            p = Pose()
            p.position.x = float(self.particles[i, 0])
            p.position.y = float(self.particles[i, 1])
            p.orientation.z = math.sin(self.particles[i, 2] / 2.0)
            p.orientation.w = math.cos(self.particles[i, 2] / 2.0)
            pa.poses.append(p)
        self.pcloud_pub.publish(pa)

        # Pose estimada con covarianza aproximada
        ps = PoseWithCovarianceStamped()
        ps.header = pa.header
        ps.pose.pose.position.x = x_m
        ps.pose.pose.position.y = y_m
        ps.pose.pose.orientation.z = math.sin(yaw_m / 2.0)
        ps.pose.pose.orientation.w = math.cos(yaw_m / 2.0)
        d_xy = self.particles[:, :2] - np.array([x_m, y_m])
        try:
            cov_xy = np.cov(d_xy.T, aweights=self.weights)
            ps.pose.covariance[0] = float(cov_xy[0, 0])
            ps.pose.covariance[1] = float(cov_xy[0, 1])
            ps.pose.covariance[6] = float(cov_xy[1, 0])
            ps.pose.covariance[7] = float(cov_xy[1, 1])
        except Exception:
            pass
        # Varianza circular para yaw
        sin_m = float(np.sum(np.sin(self.particles[:, 2]) * self.weights))
        cos_m = float(np.sum(np.cos(self.particles[:, 2]) * self.weights))
        R = math.hypot(sin_m, cos_m)
        ps.pose.covariance[35] = float(-2.0 * math.log(R)) if R > 1e-9 else 1.0
        self.pose_pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = AmclLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
