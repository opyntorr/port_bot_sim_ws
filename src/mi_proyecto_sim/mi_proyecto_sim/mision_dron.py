#!/usr/bin/env python3
"""
Mision del dron: takeoff -> 9 waypoints en grid 3x3 a z=2.5m con foto en cada uno
-> aterriza en el ultimo WP -> stitching -> binarizacion -> PGM/YAML -> detecta
ArUcos del carrito (id 4) y meta (id 5) en la imagen stitcheada.

Mapa generado: 3.9m x 3.9m con origen en el centro de la imagen.
"""

import json
import math
import subprocess
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from tf2_ros import StaticTransformBroadcaster

try:
    from tello_msgs.srv import TelloAction
    _HAS_TELLO_ACTION = True
except ImportError:
    _HAS_TELLO_ACTION = False

def make_range_points(n, min_value, max_value):
    if n == 1:
        return [(min_value + max_value) / 2]

    step = (max_value - min_value) / (n - 1)
    return [min_value + i * step for i in range(n)]

def make_grid(cols, rows):
    x_points = make_range_points(cols, -1.6, 1.6)
    y_points = make_range_points(rows, -1.3, 1.6)

    return [(x, y) for y in y_points for x in x_points]

# Patron serpiente en grid 3x3 (todos a z=2.5m). Spacing 1.3m para ~60% overlap.
# Cobertura: -1.3m a 1.3m en X e Y.
_Z = 2
WAYPOINTS = []
GRID_SIZE = 5
_COLS = make_range_points(GRID_SIZE, -1.4, 1.4)
_ROWS = list(reversed(make_range_points(GRID_SIZE, -1.2, 1.4)))

for i, y in enumerate(_ROWS):
    row = _COLS if i % 2 == 0 else list(reversed(_COLS))

    for x in row:
        WAYPOINTS.append((x, y, _Z))

MAP_SIZE_M = 3.9
ARUCO_CARRITO_ID = 4
ARUCO_META_ID = 5

POS_TOL = 0.25
POS_TOL_EXIT = 0.50   # hysteresis: solo reinicia el temporizador si se aleja más de esto
SETTLE_TIME = 1.5
TAKEOFF_Z = 0.4       # altura fusionada mínima para considerar el despegue completo
TAKEOFF_TIMEOUT = 15.0  # s en TAKEOFF sin pasar la compuerta -> abortar y aterrizar
POSE_STALE_S = 1.0    # edad máxima de /odometry/filtered antes de avisar

def _find_ws_root() -> Path:
    """Devuelve la raíz del workspace: /ros2_ws en Docker, o se deriva del share dir en el host."""
    docker = Path('/ros2_ws')
    if docker.exists():
        return docker
    # share dir: <ws>/install/<pkg>/share/<pkg>  → parents[3] == <ws>
    return Path(get_package_share_directory('mi_proyecto_sim')).parents[3]


WS_ROOT = _find_ws_root()
STITCH_SCRIPT = WS_ROOT / 'src' / 'demo_tello_sim' / 'camera_calibration' / 'stitch_images.py'
STITCH_POSICION_SCRIPT = WS_ROOT / 'src' / 'mi_proyecto_sim' / 'mi_proyecto_sim' / 'stitch_posicion.py'
MAPS_DIR = WS_ROOT / 'src' / 'mi_proyecto_sim' / 'maps'


class MisionDron(Node):
    def __init__(self):
        super().__init__('mision_dron')

        self.declare_parameter('use_real_drone', False)
        self.declare_parameter('camera_topic', '')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('optitrack_topic', '/optitrack/rigid_body')

        self.real = self.get_parameter('use_real_drone').get_parameter_value().bool_value

        # Tópicos: parámetro explícito > default según modo
        cam_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        if not cam_topic:
            cam_topic = '/image_raw' if self.real else '/drone1/camera_down'
        if not odom_topic:
            odom_topic = '/odometry/filtered' if self.real else '/drone1/odom'

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.out_dir = WS_ROOT / 'src' / 'mi_proyecto_sim' / 'mision_output' / stamp
        self.fotos_dir = self.out_dir / 'fotos'
        self.fotos_dir.mkdir(parents=True, exist_ok=True)

        self.latest_image = None
        self.current_pose = None
        self.optitrack_yaw = None

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        optitrack_topic = self.get_parameter('optitrack_topic').get_parameter_value().string_value
        self.create_subscription(Image, cam_topic, self._image_cb, qos_be)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_be)
        if self.real:
            self.create_subscription(PoseStamped, optitrack_topic, self._optitrack_cb, qos_be)
        self.target_pub = self.create_publisher(Point, '/drone1/target_position', 10)
        self.static_tf = StaticTransformBroadcaster(self)

        # Takeoff/land: servicio TelloAction (sim) o topics Empty (real)
        if self.real:
            self.takeoff_pub = self.create_publisher(Empty, '/takeoff', 1)
            self.land_pub = self.create_publisher(Empty, '/land', 1)
            self.action_cli = None
        else:
            self.takeoff_pub = None
            self.land_pub = None
            self.action_cli = self.create_client(TelloAction, '/drone1/tello_action') \
                if _HAS_TELLO_ACTION else None

        self.state = 'INIT'
        self.wp_index = 0
        self.settle_start = None
        self.takeoff_t = None
        self.pose_t = None
        self.climb_xy = None
        self.land_t = None
        self.land_called = False

        self.timer = self.create_timer(0.1, self._tick)
        modo = 'REAL' if self.real else 'SIM'
        self.get_logger().info(
            f'Mision lista [{modo}] — cam={cam_topic} odom={odom_topic} '
            f'salida={self.out_dir}'
        )

    def _image_cb(self, msg):
        self.latest_image = msg

    def _optitrack_cb(self, msg):
        if msg.header.frame_id != 'drone':
            return
        q = msg.pose.orientation
        self.optitrack_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw from quaternion: atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_pose = (p.x, p.y, p.z, yaw)
        self.pose_t = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _call_action(self, cmd):
        if self.real:
            if cmd == 'takeoff':
                self.takeoff_pub.publish(Empty())
            elif cmd == 'land':
                self.land_pub.publish(Empty())
            self.get_logger().info(f'Comando real: {cmd}')
            return True
        if self.action_cli is None:
            self.get_logger().error('TelloAction no disponible')
            return False
        if not self.action_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Servicio /drone1/tello_action no disponible')
            return False
        req = TelloAction.Request()
        req.cmd = cmd
        self.action_cli.call_async(req)
        self.get_logger().info(f'Comando sim: {cmd}')
        return True

    def _publish_target(self, x, y, z):
        self.target_pub.publish(Point(x=float(x), y=float(y), z=float(z)))

    def _distance_to(self, x, y, z):
        if self.current_pose is None:
            return float('inf')
        cx, cy, cz = self.current_pose[:3]
        return math.sqrt((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2)

    def _tick(self):
        now = self._now()
        if now == 0:
            return

        # Pose congelada: el fuser/OptiTrack dejó de publicar — todo el control está ciego
        if self.pose_t is not None and (now - self.pose_t) > POSE_STALE_S:
            self.get_logger().error(
                f'POSE CONGELADA: sin /odometry/filtered hace {now - self.pose_t:.1f} s '
                '(revisar OptiTrack / pose_fuser)',
                throttle_duration_sec=2.0,
            )

        if self.state == 'INIT':
            #self._publish_target(0.0, 0.0, _Z)
            if self.latest_image is None or self.current_pose is None:
                return
            self.get_logger().info('Camara y odom OK. Despegando...')
            self._call_action('takeoff')
            self.takeoff_t = now
            self.state = 'TAKEOFF'
            return

        if self.state == 'TAKEOFF':
            elapsed = now - self.takeoff_t
            pose_age = (now - self.pose_t) if self.pose_t is not None else float('inf')
            if self.current_pose[2] > TAKEOFF_Z and elapsed > 4.0:
                # Subir vertical en el sitio antes del primer waypoint: un tramo
                # diagonal largo a baja altura hace que el Tello pierda altitud.
                self.climb_xy = (self.current_pose[0], self.current_pose[1])
                self.get_logger().info(
                    f'En aire. Subiendo a z={_Z} en '
                    f'({self.climb_xy[0]:+.2f}, {self.climb_xy[1]:+.2f})...'
                )
                self.state = 'CLIMB'
                return
            self.get_logger().info(
                f'TAKEOFF: z_fusionada={self.current_pose[2]:.2f} m '
                f'(umbral {TAKEOFF_Z}) t={elapsed:.1f} s pose_age={pose_age:.2f} s',
                throttle_duration_sec=1.0,
            )
            if elapsed > TAKEOFF_TIMEOUT:
                self.get_logger().error(
                    f'TAKEOFF TIMEOUT tras {elapsed:.1f} s: z_fusionada='
                    f'{self.current_pose[2]:.2f} m nunca superó {TAKEOFF_Z} m '
                    f'(pose_age={pose_age:.2f} s). Abortando: aterrizaje de seguridad.'
                )
                self.state = 'LAND'
            return

        if self.state == 'CLIMB':
            cx, cy = self.climb_xy
            self._publish_target(cx, cy, _Z)
            self.get_logger().info(
                f'CLIMB: z={self.current_pose[2]:.2f} m (meta {_Z})',
                throttle_duration_sec=1.0,
            )
            if abs(self.current_pose[2] - _Z) < 0.3:
                self.get_logger().info('Altura de crucero alcanzada. Iniciando waypoints...')
                self.state = 'GOTO_WP'
                self.wp_index = 0
                self.settle_start = None
            return

        if self.state == 'GOTO_WP':
            x, y, z = WAYPOINTS[self.wp_index]
            self._publish_target(x, y, z)

            dist = self._distance_to(x, y, z)
            if dist < POS_TOL:
                if self.settle_start is None:
                    self.settle_start = now
                    self.get_logger().info(
                        f'WP{self.wp_index} dentro de tolerancia (d={dist:.3f}m). '
                        f'Esperando {SETTLE_TIME}s de estabilidad...'
                    )
                elif (now - self.settle_start) > SETTLE_TIME:
                    self._save_photo(self.wp_index)
                    self.settle_start = None
                    self.wp_index += 1
                    if self.wp_index >= len(WAYPOINTS):
                        self.get_logger().info('Todos los waypoints completados.')
                        self.state = 'LAND'
                    else:
                        self.get_logger().info(
                            f'Avanzando a WP{self.wp_index}: '
                            f'{WAYPOINTS[self.wp_index]}'
                        )
            elif dist > POS_TOL_EXIT:
                if self.settle_start is not None:
                    self.get_logger().info(
                        f'WP{self.wp_index} settle reiniciado (d={dist:.3f}m > {POS_TOL_EXIT}m)'
                    )
                self.settle_start = None
            return

        if self.state == 'LAND':
            if not self.land_called:
                self.get_logger().info('Aterrizando en el ultimo WP...')
                self._call_action('land')
                self.land_called = True
                self.land_t = now
                return
            if self.current_pose[2] < 0.2 and (now - self.land_t) > 4.0:
                self.get_logger().info('Aterrizado. Post-procesando...')
                self.state = 'POSTPROCESS'
            return

        if self.state == 'POSTPROCESS':
            self.timer.cancel()
            try:
                self._postprocess()
                self.get_logger().info('Mision completada.')
            except Exception as exc:
                self.get_logger().error(f'Post-proceso fallo: {exc}')
            self.state = 'DONE'

    def _save_photo(self, idx):
        if self.latest_image is None:
            self.get_logger().warn(f'[WP{idx}] Sin imagen disponible — foto no guardada')
            return

        # Decodificar imagen (evita cv_bridge para compatibilidad con NumPy 2.x)
        try:
            m = self.latest_image
            img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
            if m.encoding in ('rgb8', 'RGB8'):
                img = img[:, :, ::-1].copy()
            elif m.encoding in ('mono8', 'MONO8'):
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        except Exception as exc:
            self.get_logger().error(f'[WP{idx}] Error decodificando imagen: {exc}')
            return

        h, w = img.shape[:2]
        self.get_logger().info(f'[WP{idx}] Imagen capturada: {w}x{h} px')

        # Guardar PNG
        path = self.fotos_dir / f'wp_{idx:02d}.png'
        cv2.imwrite(str(path), img)

        # Pose: pose fusionada si está disponible, waypoint nominal como fallback
        pose_src = 'nominal'
        ox, oy, oz = WAYPOINTS[idx]
        oyaw = 0.0
        if self.current_pose is not None:
            ox, oy, oz, oyaw = self.current_pose
            pose_src = 'odometria'
        if self.optitrack_yaw is not None:
            oyaw = self.optitrack_yaw
            pose_src = pose_src + '+optitrack_yaw' if pose_src != 'nominal' else 'optitrack_yaw'

        meta = {
            'wp_index': idx,
            'x':        float(ox),
            'y':        float(oy),
            'z':        float(oz),
            'yaw_deg':  float(math.degrees(oyaw)),
            'pose_src': pose_src,
            'stamp':    datetime.now().strftime('%Y%m%d_%H%M%S'),
            'img_w':    w,
            'img_h':    h,
        }

        # Guardar JSON de metadatos
        json_path = path.with_suffix('.json')
        try:
            with open(json_path, 'w') as f:
                json.dump(meta, f, indent=2)
            self.get_logger().info(
                f'[WP{idx}] JSON guardado: {json_path.name} — '
                f'pose=({ox:+.3f}, {oy:+.3f}, {oz:.3f}) '
                f'yaw={math.degrees(oyaw):+.1f}° [{pose_src}]'
            )
        except Exception as exc:
            self.get_logger().error(f'[WP{idx}] Error guardando JSON: {exc}')

    def _postprocess(self):
        stitch_out = self.out_dir / 'stitching'
        stitch_out.mkdir(exist_ok=True)
        cmd = [
            'python3', str(STITCH_POSICION_SCRIPT),
            '--input', str(self.fotos_dir),
            '--output', str(stitch_out),
            '--no-undistort',
            '--max-dist', '1.6',
            '--positional-only',   # tile grid is repetitive; SIFT causes ghost matches
        ]
        self.get_logger().info(f'Stitching: {" ".join(cmd)}')
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f'Stitching fallo:\n{result.stderr}')
            return

        panorama_path = stitch_out / 'panorama.png'
        if not panorama_path.exists():
            self.get_logger().error(f'No se encontro {panorama_path}')
            return
        self.get_logger().info(f'Panorama: {panorama_path}')

        stitched = cv2.imread(str(panorama_path))

        cart_meta = self._detect_aruco_positions(stitched)

        # Color-based binarization: dark floor tiles + orange grid lines = free;
        # everything else within coverage = obstacle (walls, boxes, etc.)
        coverage_img = cv2.imread(str(stitch_out / 'coverage.png'), cv2.IMREAD_GRAYSCALE)
        covered = (coverage_img > 127) if coverage_img is not None else \
                  (cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY) > 5)

        hsv = cv2.cvtColor(stitched, cv2.COLOR_BGR2HSV)
        floor_dark   = cv2.inRange(hsv, (0,   0,  0), (180, 80, 70))   # near-black tiles
        floor_orange = cv2.inRange(hsv, (10, 100, 80), (30, 255, 220))  # orange separators
        floor = cv2.bitwise_or(floor_dark, floor_orange)
        k3 = np.ones((3, 3), np.uint8)
        floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, k3, iterations=2)

        # ROS2 trinary occupancy: 254=free, 0=occupied, 205=unknown
        occ = np.full(stitched.shape[:2], 205, dtype=np.uint8)
        occ[covered & (floor > 0)] = 254
        occ[covered & (floor == 0)] = 0
        binary = occ

        MAPS_DIR.mkdir(parents=True, exist_ok=True)
        pgm_path = MAPS_DIR / 'mapa_mision.pgm'
        yaml_path = MAPS_DIR / 'mapa_mision.yaml'
        cv2.imwrite(str(pgm_path), binary)

        h, w = binary.shape
        scale_path = stitch_out / 'scale.json'
        if scale_path.exists():
            sd = json.loads(scale_path.read_text())
            resolution = 1.0 / sd['pixels_per_meter']
            origin_x   = sd['origin_x_m']
            origin_y   = sd['origin_y_m']
        else:
            resolution = MAP_SIZE_M / max(w, h)
            origin_x   = -MAP_SIZE_M / 2.0
            origin_y   = -MAP_SIZE_M / 2.0

        with open(yaml_path, 'w') as f:
            yaml.dump({
                'image':           pgm_path.name,
                'mode':            'trinary',
                'resolution':      float(resolution),
                'origin':          [float(origin_x), float(origin_y), 0.0],
                'negate':          0,
                'occupied_thresh': 0.65,
                'free_thresh':     0.196,
            }, f)
        self.get_logger().info(f'Mapa: {pgm_path} ({w}x{h}, res={resolution:.4f} m/px)')

        if cart_meta:
            self._publish_aruco_tfs(cart_meta, w, h, resolution)
            with open(self.out_dir / 'arucos.yaml', 'w') as f:
                yaml.dump(cart_meta, f)

    def _detect_aruco_positions(self, image):
        try:
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
        except AttributeError:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, _ = detector.detectMarkers(image)

        results = {}
        if ids is None:
            self.get_logger().warn('Ningun ArUco detectado en el panorama')
            return results
        ids_flat = ids.flatten().tolist()
        for target, name in [(ARUCO_CARRITO_ID, 'carrito'), (ARUCO_META_ID, 'meta')]:
            if target not in ids_flat:
                self.get_logger().warn(f'ArUco id={target} ({name}) no detectado')
                continue
            i = ids_flat.index(target)
            pts = corners[i][0]
            cx = float(np.mean(pts[:, 0]))
            cy = float(np.mean(pts[:, 1]))
            dx = pts[1][0] - pts[0][0]
            dy = pts[1][1] - pts[0][1]
            yaw_img = math.atan2(dy, dx)
            results[name] = {
                'id': int(target),
                'px_x': cx,
                'px_y': cy,
                'yaw_img': float(yaw_img),
            }
            self.get_logger().info(f'ArUco {name} (id={target}) px=({cx:.0f},{cy:.0f})')
        return results

    def _publish_aruco_tfs(self, cart_meta, img_w, img_h, resolution):
        cx_img = img_w / 2.0
        cy_img = img_h / 2.0
        transforms = []
        for name, data in cart_meta.items():
            mx = (data['px_x'] - cx_img) * resolution
            my = -(data['px_y'] - cy_img) * resolution
            yaw = -data['yaw_img']

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = f'{name}_aruco'
            t.transform.translation.x = float(mx)
            t.transform.translation.y = float(my)
            t.transform.translation.z = 0.0
            sy = math.sin(yaw * 0.5)
            cy_q = math.cos(yaw * 0.5)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = float(sy)
            t.transform.rotation.w = float(cy_q)
            transforms.append(t)
            self.get_logger().info(
                f'TF map->{name}_aruco @ ({mx:+.2f},{my:+.2f}) yaw={math.degrees(yaw):+.1f}'
            )
        self.static_tf.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = MisionDron()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
