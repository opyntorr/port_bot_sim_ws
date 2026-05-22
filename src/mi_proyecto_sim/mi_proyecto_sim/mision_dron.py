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
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
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

def make_range_points(n, min_value=-1.4, max_value=1.4):
    step = (max_value - min_value) / (n - 1)
    return [min_value + i * step for i in range(n)]

# Patron serpiente en grid 3x3 (todos a z=2.5m). Spacing 1.3m para ~60% overlap.
# Cobertura: -1.3m a 1.3m en X e Y.
_Z = 2.2
WAYPOINTS = []
GRID_SIZE = 4
_COLS = make_range_points(GRID_SIZE)
_ROWS = list(reversed(make_range_points(GRID_SIZE)))
for i, y in enumerate(_ROWS):
    row = _COLS if i % 2 == 0 else list(reversed(_COLS))
    for x in row:
        WAYPOINTS.append((x, y, _Z))

MAP_SIZE_M = 3.9
ARUCO_CARRITO_ID = 4
ARUCO_META_ID = 0

POS_TOL = 0.25
POS_TOL_EXIT = 0.50   # hysteresis: solo reinicia el temporizador si se aleja más de esto
SETTLE_TIME = 1.5

def _find_ws_root() -> Path:
    """Devuelve la raíz del workspace: /ros2_ws en Docker, o se deriva del share dir en el host."""
    docker = Path('/ros2_ws')
    if docker.exists():
        return docker
    # share dir: <ws>/install/<pkg>/share/<pkg>  → parents[3] == <ws>
    return Path(get_package_share_directory('mi_proyecto_sim')).parents[3]


WS_ROOT = _find_ws_root()
STITCH_SCRIPT = WS_ROOT / 'src' / 'demo_tello_sim' / 'camera_calibration' / 'stitch_images.py'
STITCH_POSICION_PKG_DIR = WS_ROOT / 'src' / 'mi_proyecto_sim'  # cwd para `python3 -m mision.stitch_pose`
MAPS_DIR = WS_ROOT / 'src' / 'mi_proyecto_sim' / 'maps'


class MisionDron(Node):
    def __init__(self):
        super().__init__('mision_dron')
        self.get_logger().info('=== INICIANDO MISION DRON (VERSION CON FALLBACK FIX Y DEBUG) ===')

        self.declare_parameter('use_real_drone', False)
        self.declare_parameter('camera_topic', '')
        self.declare_parameter('odom_topic', '')
        self.declare_parameter('optitrack_topic', '/optitrack/rigid_body')
        self.declare_parameter('invert_colors', False)
        self.declare_parameter('map_size_m', 3.9)

        self.real = self.get_parameter('use_real_drone').get_parameter_value().bool_value
        self.invert_colors = self.get_parameter('invert_colors').get_parameter_value().bool_value
        self.map_size_m = self.get_parameter('map_size_m').get_parameter_value().double_value

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
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.emergency_pub = self.create_publisher(Empty, '/emergency', 1)
        self.static_tf = StaticTransformBroadcaster(self)

        # Inicializar parametros de camara y detector ArUco para Visual Servoing
        camera_yaml = WS_ROOT / 'src' / 'mi_proyecto_sim' / 'config' / ('camera_tello.yaml' if self.real else 'camera_tello_sim.yaml')
        with open(camera_yaml, 'r') as f:
            cam_data = yaml.safe_load(f)
        self.K = np.array(cam_data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
        self.D = np.array(cam_data['distortion_coefficients']['data'], dtype=np.float64).flatten()
        self.w_cam = int(cam_data['image_width'])
        self.h_cam = int(cam_data['image_height'])
        self.newK, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (self.w_cam, self.h_cam), alpha=0.0)

        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_detector = None
        except AttributeError:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.REST_ARUCO_ID = 1
        self.REST_ARUCO_NOMINAL_X = 0.0
        self.REST_ARUCO_NOMINAL_Y = -1.4

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
        self.land_t = None
        self.land_called = False
        self.last_saved_stamp = None  # evita guardar el mismo frame en WPs consecutivos

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

        # Fallback de 20 segundos despues de terminar los waypoints
        if hasattr(self, 'last_wp_time') and self.state not in ['LAND', 'POSTPROCESS', 'DONE']:
            if (now - self.last_wp_time) > 20.0:
                self.get_logger().warn(f'Han pasado 20 segundos desde el ultimo WP. Estado actual: {self.state}. Forzando aterrizaje!')
                self.state = 'LAND'
                self.land_called = False
                self.land_t = now
                
                self.emergency_pub.publish(Empty())  # Apaga el PID para que no evite el aterrizaje
                self.cmd_vel_pub.publish(Twist())
                return

        if self.state == 'INIT':
            if self.latest_image is None or self.current_pose is None:
                return
            self.get_logger().info('Camara y odom OK. Despegando...')
            self._call_action('takeoff')
            self.takeoff_t = now
            self.state = 'TAKEOFF'
            return

        if self.state == 'TAKEOFF':
            if self.current_pose[2] > 0.8 and (now - self.takeoff_t) > 4.0:
                self.get_logger().info('En aire. Iniciando waypoints...')
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
                    saved = self._save_photo(self.wp_index)
                    if saved is False:
                        # Frame aún no fresco; intenta de nuevo en el siguiente tick
                        return
                    self.settle_start = None
                    self.wp_index += 1
                    if self.wp_index >= len(WAYPOINTS):
                        self.get_logger().info('Todos los waypoints completados. Procesando fotos para encontrar el ArUco de reposo...')
                        self.last_wp_time = now
                        
                        # Buscar el ArUco de reposo en las fotos tomadas
                        detections = self._detect_aruco_positions_from_photos()
                        aruco_name = f'aruco_{self.REST_ARUCO_ID}'
                        
                        if aruco_name in detections:
                            self.REST_ARUCO_NOMINAL_X = detections[aruco_name]['world_x']
                            self.REST_ARUCO_NOMINAL_Y = detections[aruco_name]['world_y']
                            self.get_logger().info(f"ArUco de reposo encontrado en fotos: ({self.REST_ARUCO_NOMINAL_X:.2f}, {self.REST_ARUCO_NOMINAL_Y:.2f})")
                            self.state = 'GOTO_REST_ARUCO'
                            self.servo_substate = 'CENTERING'
                            self.settle_start = None
                        else:
                            self.get_logger().warn("ArUco de reposo NO encontrado en fotos. Pasando directo a busqueda visual (20s max).")
                            self.state = 'VISUAL_SERVO'
                            self.servo_substate = 'CENTERING'
                            self.emergency_pub.publish(Empty())
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

        if self.state == 'GOTO_REST_ARUCO':
            self._publish_target(self.REST_ARUCO_NOMINAL_X, self.REST_ARUCO_NOMINAL_Y, _Z)
            dist = self._distance_to(self.REST_ARUCO_NOMINAL_X, self.REST_ARUCO_NOMINAL_Y, _Z)
            self.get_logger().info(f'[DEBUG] GOTO_REST_ARUCO: dist={dist:.3f}, settle={self.settle_start}', throttle_duration_sec=0.5)
            if dist < POS_TOL:
                if self.settle_start is None:
                    self.settle_start = now
                elif (now - self.settle_start) > 0.5:  # Transicion rapida al servo visual
                    self.get_logger().info('Cerca del Aruco de reposo. Desactivando PID e iniciando control visual...')
                    self.emergency_pub.publish(Empty())  # Apaga el position_controller
                    self.state = 'VISUAL_SERVO'
            else:
                # Si se sale del radio de tolerancia, resetear settle_start para que deba volver a estabilizarse
                if self.settle_start is not None:
                    self.get_logger().info(f'[DEBUG] Settle reset en GOTO_REST_ARUCO porque dist={dist:.3f} >= POS_TOL')
                self.settle_start = None
            return

        if self.state == 'VISUAL_SERVO':
            if self.latest_image is None:
                return
            
            # Decodificar imagen para el servo
            m = self.latest_image
            img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
            if m.encoding in ('mono8', 'MONO8'):
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            
            img_undist = cv2.undistort(img, self.K, self.D, None, self.newK)
            
            if self.aruco_detector is not None:
                corners, ids, _ = self.aruco_detector.detectMarkers(img_undist)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(img_undist, self.aruco_dict, parameters=self.aruco_params)
            
            twist = Twist()
            aruco_found = False
            
            if ids is not None:
                ids_flat = ids.flatten().tolist()
                if self.REST_ARUCO_ID in ids_flat:
                    idx = ids_flat.index(self.REST_ARUCO_ID)
                    pts = corners[idx][0]
                    cx = np.mean(pts[:, 0])
                    cy = np.mean(pts[:, 1])
                    
                    # Error en pixeles respecto al centro
                    err_x = cx - self.w_cam / 2.0
                    err_y = cy - self.h_cam / 2.0
                    
                    # Error de orientacion
                    dx = pts[1][0] - pts[0][0]
                    dy = pts[1][1] - pts[0][1]
                    yaw_img = math.atan2(dy, dx)
                    
                    kp_xy = 0.002
                    kp_yaw = 0.6
                    
                    cmd_x = -err_y * kp_xy
                    cmd_y = -err_x * kp_xy 
                    err_yaw = -yaw_img 
                    
                    cmd_yaw = err_yaw * kp_yaw
                    
                    max_xy = 0.2
                    max_yaw = 0.5
                    dist_pix = math.hypot(err_x, err_y)
                    
                    if self.servo_substate == 'CENTERING':
                        # Fase 1: Centrar XY y bajar a 25 cm (0.25m) SIN rotar
                        twist.linear.x = max(min(cmd_x, max_xy), -max_xy)
                        twist.linear.y = max(min(cmd_y, max_xy), -max_xy)
                        twist.angular.z = 0.0
                        
                        target_z = 0.35  # Aproximadamente 15-20cm sobre el ArUco
                        if self.current_pose[2] > target_z + 0.1:
                            twist.linear.z = -0.15
                        elif self.current_pose[2] < target_z - 0.1:
                            twist.linear.z = 0.15
                        else:
                            twist.linear.z = 0.0
                            
                        self.get_logger().info(f'[Fase 1] Centrando: err_xy={dist_pix:.1f}px, Z={self.current_pose[2]:.2f}m', throttle_duration_sec=0.5)
                        
                        if dist_pix < 25 and abs(self.current_pose[2] - target_z) < 0.15:
                            self.get_logger().info('Centrado en XY. Pasando a rotacion (YAW).')
                            self.servo_substate = 'YAWING'
                            
                    elif self.servo_substate == 'YAWING':
                        # Fase 2: Mantener posicion XY y rotar para igualar orientacion
                        twist.linear.x = max(min(cmd_x, max_xy*0.5), -max_xy*0.5)
                        twist.linear.y = max(min(cmd_y, max_xy*0.5), -max_xy*0.5)
                        twist.angular.z = max(min(cmd_yaw, max_yaw), -max_yaw)
                        
                        # Mantener altura
                        target_z = 0.35
                        if self.current_pose[2] > target_z + 0.05:
                            twist.linear.z = -0.1
                        elif self.current_pose[2] < target_z - 0.05:
                            twist.linear.z = 0.1
                        else:
                            twist.linear.z = 0.0
                            
                        self.get_logger().info(f'[Fase 2] Rotando: err_yaw={math.degrees(err_yaw):.1f}deg', throttle_duration_sec=0.5)
                        
                        if abs(err_yaw) < math.radians(5) and dist_pix < 30:
                            self.get_logger().info('Perfectamente alineado. Aterrizando!')
                            self.state = 'LAND'
                            self.land_called = False
                            self.land_t = now
                            self.cmd_vel_pub.publish(Twist())
                            return
                    
                    aruco_found = True
            
            if not aruco_found:
                self.get_logger().info('ArUco 1 no visible, bajando y buscando...', throttle_duration_sec=1.0)
                twist.linear.z = -0.15  # Bajar buscando
                twist.angular.z = 0.0   # No girar para no marear
                
            self.cmd_vel_pub.publish(twist)
            return

        if self.state == 'LAND':
            if not self.land_called:
                self.get_logger().info('Ejecutando comando land...')
                self._call_action('land')
                self.land_called = True
                self.land_t = now
                return
            if self.current_pose[2] < 0.2 and (now - self.land_t) > 4.0:
                self.get_logger().info('Aterrizado. Post-procesando...')
                self.state = 'POSTPROCESS'
            elif (now - self.land_t) > 8.0:
                self.get_logger().info('Timeout aterrizaje. Post-procesando...')
                self.state = 'POSTPROCESS'
            return

        if self.state == 'POSTPROCESS':
            self.timer.cancel()
            try:
                self._postprocess()
                self.get_logger().info('Mision completada. Cerrando nodo...')
            except Exception as exc:
                self.get_logger().error(f'Post-proceso fallo: {exc}')
            self.state = 'DONE'
            raise SystemExit

    def _save_photo(self, idx):
        if self.latest_image is None:
            self.get_logger().warn(f'[WP{idx}] Sin imagen disponible — foto no guardada')
            return

        # Verifica que el frame sea NUEVO respecto a la última foto guardada;
        # si la cámara va lenta y no hay frame fresco, no avances todavía.
        stamp = self.latest_image.header.stamp
        stamp_key = (stamp.sec, stamp.nanosec)
        if self.last_saved_stamp is not None and stamp_key == self.last_saved_stamp:
            self.get_logger().warn(
                f'[WP{idx}] Frame de cámara aún no actualizado (mismo stamp que WP previo) — espero'
            )
            return False
        self.last_saved_stamp = stamp_key

        # Decodificar imagen (evita cv_bridge para compatibilidad con NumPy 2.x)
        try:
            m = self.latest_image
            img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
            if m.encoding in ('rgb8', 'RGB8'):
                if not self.invert_colors:
                    # convención normal: RGB→BGR para que cv2.imwrite guarde bien
                    img = img[:, :, ::-1].copy()
                # else: dejar bytes en RGB; cv2.imwrite los interpreta como BGR
                # y el PNG queda con R↔B invertido (replica el Tello real).
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
            'python3', '-m', 'mision.stitch_pose',
            '--input', str(self.fotos_dir),
            '--output', str(stitch_out),
            '--map-name', 'occupancy_map',
        ]
        self.get_logger().info(f'Stitching: {" ".join(cmd)} (cwd={STITCH_POSICION_PKG_DIR})')
        result = subprocess.run(
            cmd, capture_output=True, text=True,
            cwd=str(STITCH_POSICION_PKG_DIR),
        )
        if result.returncode != 0:
            self.get_logger().error(f'Stitching fallo:\n{result.stderr}')
            return

        panorama_path = stitch_out / 'mosaic_pose.png'
        if not panorama_path.exists():
            self.get_logger().error(f'No se encontro {panorama_path}')
            return
        self.get_logger().info(f'Panorama: {panorama_path}')

        stitched = cv2.imread(str(panorama_path))

        cart_meta = self._detect_aruco_positions_from_photos()

        # Usar directamente el mapa binarizado que genero stitch_pose.py
        # (su deteccion de obstaculos por color es mas precisa)
        stitch_pgm = stitch_out / 'occupancy_map.pgm'
        stitch_yaml_path = stitch_out / 'occupancy_map.yaml'

        if stitch_pgm.exists():
            binary = cv2.imread(str(stitch_pgm), cv2.IMREAD_GRAYSCALE)
            # El PGM del stitcher esta en formato ROS (row0=min_y, ya volteado).
            # slam_occupancy_grid lo voltea de nuevo al cargarlo, asi que
            # lo revertimos aqui para que el doble-flip resulte correcto.
            binary = cv2.flip(binary, 0)
            self.get_logger().info(f'Usando mapa del stitcher: {stitch_pgm}')
        else:
            # Fallback: binarizacion simple si el stitcher no genero PGM
            self.get_logger().warn('No se encontro PGM del stitcher. Binarizando el mosaico...')
            gray = cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY)
            covered = gray > 5
            binary = np.full(gray.shape, 205, dtype=np.uint8)
            binary[covered & (gray > 127)] = 254
            binary[covered & (gray <= 127)] = 0

        MAPS_DIR.mkdir(parents=True, exist_ok=True)
        pgm_path = MAPS_DIR / 'mapa_mision.pgm'
        yaml_path = MAPS_DIR / 'mapa_mision.yaml'
        cv2.imwrite(str(pgm_path), binary)

        h, w = binary.shape
        if stitch_yaml_path.exists():
            sd = yaml.safe_load(stitch_yaml_path.read_text())
            resolution = float(sd['resolution'])
            origin_x   = float(sd['origin'][0])
            origin_y   = float(sd['origin'][1])
        else:
            resolution = self.map_size_m / max(w, h)
            origin_x   = -self.map_size_m / 2.0
            origin_y   = -self.map_size_m / 2.0

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

        # Copiar todos los artefactos del stitching a la carpeta de mapas
        import shutil
        for artifact in stitch_out.glob('*'):
            dest = MAPS_DIR / artifact.name
            if artifact.is_file():
                shutil.copy2(str(artifact), str(dest))
        self.get_logger().info(f'Artefactos del stitching copiados a {MAPS_DIR}')

        if cart_meta:
            # Guardar arucos.yaml junto al mapa para que el nodo publicador_tfs lo lea
            arucos_yaml_path = MAPS_DIR / 'arucos.yaml'
            with open(arucos_yaml_path, 'w') as f:
                yaml.dump(cart_meta, f)
            self.get_logger().info(f'ArUcos guardados en {arucos_yaml_path}')
            # Tambien guardar copia en el directorio de salida de la mision
            with open(self.out_dir / 'arucos.yaml', 'w') as f:
                yaml.dump(cart_meta, f)
                
            # Dibujar ArUcos visualmente en el mapa final
            grid_path = stitch_out / 'occupancy_map_grid.png'
            if grid_path.exists():
                grid_img = cv2.imread(str(grid_path))
                for name, data in cart_meta.items():
                    # Convertir coordenadas de mundo (metros) a pixeles del mapa
                    px_x = int((data['world_x'] - origin_x) / resolution)
                    # En ROS el origen Y es abajo, pero en OpenCV es arriba
                    px_y = grid_img.shape[0] - int((data['world_y'] - origin_y) / resolution)
                    
                    # Dibujar un punto rojo y el nombre
                    cv2.circle(grid_img, (px_x, px_y), 15, (0, 0, 255), -1)
                    cv2.putText(grid_img, f"{name} (id:{data['id']})", (px_x + 20, px_y + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
                
                # Guardar en stitching y en maps
                arucos_vis_path = stitch_out / 'occupancy_map_grid_arucos.png'
                cv2.imwrite(str(arucos_vis_path), grid_img)
                shutil.copy2(str(arucos_vis_path), str(MAPS_DIR / arucos_vis_path.name))
                self.get_logger().info(f"ArUcos dibujados en {arucos_vis_path.name}")

    def _detect_aruco_positions_from_photos(self):
        camera_yaml = WS_ROOT / 'src' / 'mi_proyecto_sim' / 'config' / ('camera_tello.yaml' if self.real else 'camera_tello_sim.yaml')
        with open(camera_yaml, 'r') as f:
            cam_data = yaml.safe_load(f)
        
        K = np.array(cam_data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
        D = np.array(cam_data['distortion_coefficients']['data'], dtype=np.float64).flatten()
        w_cam = int(cam_data['image_width'])
        h_cam = int(cam_data['image_height'])
        cam_rot = math.radians(float(cam_data.get('camera_rotation_deg', 0.0)))
        
        newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w_cam, h_cam), alpha=0.0)
        fx, fy = newK[0, 0], newK[1, 1]
        cx_cam, cy_cam = newK[0, 2], newK[1, 2]

        try:
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters_create()
            detector = None
        except AttributeError:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        best_detections = {}

        for img_path in sorted(self.fotos_dir.glob('wp_*.png')):
            json_path = img_path.with_suffix('.json')
            if not json_path.exists():
                continue
            
            with open(json_path, 'r') as f:
                meta = json.load(f)
                
            img = cv2.imread(str(img_path))
            if img is None:
                continue
                
            img_undistorted = cv2.undistort(img, K, D, None, newK)
            
            if detector is not None:
                corners, ids, _ = detector.detectMarkers(img_undistorted)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(img_undistorted, aruco_dict, parameters=parameters)
                
            if ids is None:
                continue
                
            ids_flat = ids.flatten().tolist()
            altitude = float(meta['z'])
            wp_x = float(meta['x'])
            wp_y = float(meta['y'])
            yaw_rad = math.radians(float(meta['yaw_deg']))
            theta = yaw_rad + cam_rot

            known_names = {ARUCO_CARRITO_ID: 'carrito', ARUCO_META_ID: 'meta'}
            
            for idx, target in enumerate(ids_flat):
                name = known_names.get(target, f'aruco_{target}')
                
                pts = corners[idx][0]
                u = float(np.mean(pts[:, 0]))
                v = float(np.mean(pts[:, 1]))
                
                # Distancia al centro de la imagen
                dist_to_center = math.hypot(u - w_cam/2.0, v - h_cam/2.0)
                
                if name not in best_detections or dist_to_center < best_detections[name]['dist']:
                    x_cam = (u - cx_cam) * altitude / fx
                    y_cam = (v - cy_cam) * altitude / fy
                    
                    dx_rot = x_cam * math.cos(theta) + y_cam * math.sin(theta)
                    dy_rot = -x_cam * math.sin(theta) + y_cam * math.cos(theta)
                    
                    world_x = wp_x + dx_rot
                    world_y = wp_y - dy_rot
                    
                    dx = pts[1][0] - pts[0][0]
                    dy = pts[1][1] - pts[0][1]
                    yaw_img = math.atan2(dy, dx)
                    # Offset fisico entre el eje "esquina 0 -> esquina 1" del ArUco
                    # y el "forward" real del objeto en el mundo. Cada marker puede
                    # tener su textura rotada distinto, por eso es por-ID.
                    # Calibrar una vez por marker: poner el objeto a yaw=0, detectar,
                    # y el yaw_world resultante (con signo invertido) es el offset.
                    ARUCO_YAW_OFFSETS = {
                        ARUCO_CARRITO_ID: -math.pi / 2,
                        ARUCO_META_ID: -math.pi,
                    }
                    offset = ARUCO_YAW_OFFSETS.get(target, 0.0)
                    yaw_world = -yaw_img - theta + offset
                    
                    best_detections[name] = {
                        'id': int(target),
                        'world_x': float(world_x),
                        'world_y': float(world_y),
                        'yaw_world': float(yaw_world),
                        'dist': dist_to_center,
                        'source_img': img_path.name
                    }
                        
        for name, data in best_detections.items():
            self.get_logger().info(f"ArUco {name} (id={data['id']}) detectado en {data['source_img']} a dist={data['dist']:.1f}px -> MUNDO: ({data['world_x']:.2f}, {data['world_y']:.2f})")
            
        return best_detections

    # OBSOLETO: publicaba ArUcos en frame `map` directamente con coords absolutas
    # del stitching, lo cual chocaba con el frame `map` de slam_toolbox (que esta
    # anclado al carrito). publicador_tfs_arucos ahora hace la transformacion al
    # frame del carrito y publica todas las TFs estaticas. No re-habilitar este
    # metodo sin coordinar con publicador_tfs_arucos.
    # def _publish_aruco_tfs(self, cart_meta):
    #     transforms = []
    #     for name, data in cart_meta.items():
    #         t = TransformStamped()
    #         t.header.stamp = self.get_clock().now().to_msg()
    #         t.header.frame_id = 'map'
    #         t.child_frame_id = f'{name}_aruco'
    #         t.transform.translation.x = data['world_x']
    #         t.transform.translation.y = data['world_y']
    #         t.transform.translation.z = 0.0
    #
    #         yaw = data['yaw_world']
    #         sy = math.sin(yaw * 0.5)
    #         cy_q = math.cos(yaw * 0.5)
    #         t.transform.rotation.x = 0.0
    #         t.transform.rotation.y = 0.0
    #         t.transform.rotation.z = float(sy)
    #         t.transform.rotation.w = float(cy_q)
    #         transforms.append(t)
    #     self.static_tf.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = MisionDron()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass  # Normal exit after mission complete
    except KeyboardInterrupt:
        # Recuperacion ante Ctrl+C
        if getattr(node, 'state', None) != 'DONE':
            fotos = list(node.fotos_dir.glob('wp_*.png')) if getattr(node, 'fotos_dir', None) else []
            if len(fotos) > 0:
                node.get_logger().warn(f'Interrupcion (Ctrl+C) detectada! Tienes {len(fotos)} fotos guardadas.')
                node.get_logger().warn('Generando el mapa final de emergencia. POR FAVOR NO PRESIONES CTRL+C OTRA VEZ...')
                try:
                    node._postprocess()
                    node.get_logger().info('Mapa de emergencia generado exitosamente. Cerrando.')
                except Exception as e:
                    node.get_logger().error(f'Error al generar mapa de emergencia: {e}')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
