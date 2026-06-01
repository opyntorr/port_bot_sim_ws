#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Bool
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformBroadcaster
import tf2_ros
import tf_transformations
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ControlTrayectoria(Node):
    def __init__(self):
        super().__init__('control_trayectoria')

        # El parametro use_sim_time se gestiona automaticamente por ROS 2 si se pasa en el comando.

        # Cuando es True, al terminar el approach terminal el robot se detiene
        # en vez de pasar a visual_search/visual_servo (estacionamiento en cubo).
        # Pensado para navegacion manual con goal puesto desde RViz.
        self.declare_parameter('disable_visual_modes', False)
        self.disable_visual_modes = self.get_parameter('disable_visual_modes').value
        
        # Configurar QoS Transient Local para poder leer la ruta publicada previamente
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Suscriptor al path
        self.path_sub = self.create_subscription(Path, '/rrt_path', self.path_callback, qos_profile)
        self.path_points = []
        
        # Publicador de comandos de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Lector de transformaciones TF2 (para ArUco)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)
        
        # Perfil de QoS optimizado para Odometría (Best Effort para Ignition Gazebo 6)
        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_pose)
        self._x_o = None
        self._y_o = None
        self._theta_o = None
        
        # Parametros del controlador de Kelly & Diaz (Sintonizados para robustez ante deriva)
        self.h = 0.25          # Aumentado (antes 0.1) para absorber el ruido y la deriva sin sobreaccionar
        self.k_px = 1.5        # Ganancia proporcional bajada para estabilidad
        self.k_py = 1.5        
        self.v_max = 0.35      # Velocidad maxima permitida
        self.w_max = 1.0       # Velocidad angular suave

        # Variables para suavizado (Filtro Pasa-Baja)
        self.v_prev = 0.0
        self.w_prev = 0.0


        
        self.lookahead_dist = 0.12 # Menos atajo en curvas para más precisión
        
        self.current_target_index = 0
        self.ruta_completada = False
        self.pure_rotation_mode = False
        self._check_initial_alignment = False  # Chequeo one-shot al recibir la ruta
        self.rot_integral = 0.0 # Acumulador integral para rotación pura
        self.visual_search_mode = False
        self.visual_servo_mode = False
        self.vs_v_integral = 0.0
        self.vs_w_integral = 0.0
        self.vs_strafe_integral = 0.0
        self.goal_yaw = None  # Orientacion objetivo (se extrae del ultimo punto del Path)

        # Fase de aproximacion terminal (go-to-point sobre el centro del robot)
        # Sustituye a Kelly & Diaz en los ultimos cm para que el centro alcance
        # la meta (Kelly & Diaz controla el punto desplazado x_c y siempre deja
        # al centro h metros corto del waypoint final).
        self.terminal_approach_mode = False
        self.terminal_threshold = 0.80    # m, entra a terminal cuando el centro esta a esta dist
        self.terminal_arrival = 0.60      # m, salta a visual_search al llegar a esta dist
        self.terminal_yaw_tol = 0.10      # rad, tolerancia de alineacion al goal
        self.k_v_terminal = 0.6           # ganancia P sobre distancia
        self.v_max_terminal = 0.12        # m/s, mas suave que v_max para precision
        self.k_w_terminal = 1.2           # ganancia P de rotacion
        self.w_min_terminal = 0.30        # rad/s, minimo para vencer friccion
        
        # Variables para replanificación automática (Watchdog)
        self.replan_pub = self.create_publisher(Empty, '/replan_request', 10)
        self.last_advance_time = None  # Se inicializa cuando el reloj es válido
        self.last_target_index = 0
        self.replan_cooldown = 2.0 # Segundos de espera mínima entre peticiones
        self.odom_offset_x = 0.0
        self.odom_offset_y = 0.0
        self.odom_offset_theta = 0.0
        self.last_aruco_timestamp = None
        self.first_aruco_received = False
        self.start_time = None  # Se inicializa cuando el reloj es válido
        self.forced_odom_start = False
        
        # Guard: esperar a que use_sim_time reciba /clock válido antes de operar
        self._clock_initialized = False

        # Guard: esperar a que publicador_tfs_arucos confirme alineacion
        # mapa-SLAM antes de mover el carrito. Sin esto, hay race condition:
        # SLAM tarda ~1-3s en publicar su primera TF, durante ese tiempo el
        # carrito ya puede haber recibido una ruta y movido, invalidando el
        # alineamiento (que asume cart cerca del origen SLAM).
        self._alignment_ready = False
        qos_latch = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.alignment_sub = self.create_subscription(
            Bool, '/alignment_ready', self._alignment_callback, qos_latch
        )
        
        # Estado del LiDAR y evasión
        self.repulsion_x = 0.0
        self.repulsion_y = 0.0
        self.obstaculo_cerca = False
        self.peligro_frontal = False
        self.min_dist_frontal = 1.5 # Ampliado a 1.5 para poder medir hasta 1.0m
        self.umbral_frontal = 0.35 
        self.umbral_lateral = 0.25 
        
        self.last_log_time = None  # Se inicializa cuando el reloj es válido

        
        qos_scan = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_callback, qos_scan)
        
        # Suscriptor a la camara del carrito
        self.bridge = CvBridge()
        self.cam_sub = self.create_subscription(Image, '/cam_1/image', self.cam_callback, 1)
        # Publicador de la imagen con ArUcos dibujados
        self.image_pub = self.create_publisher(Image, '/cam_1/image_aruco', 1)
        self.marker_cx = None
        self.marker_area = None
        self.marker_asymmetry = None  # +: robot a la derecha del perpendicular al cubo
        self.img_w = 640
        
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None
        except AttributeError:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Bucle de control a 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    def _alignment_callback(self, msg):
        if msg.data and not self._alignment_ready:
            self.get_logger().info(
                'Alineacion mapa-SLAM confirmada. Habilitando control de trayectoria.'
            )
        self._alignment_ready = bool(msg.data)

    def path_callback(self, msg):
        self.get_logger().info("Ruta recibida. Iniciando seguimiento...")
        self.path_points = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.path_points.append((x, y))
        
        # Extraer orientacion del ultimo punto (publicada por el planificador RRT)
        if msg.poses:
            last_pose = msg.poses[-1].pose
            q = last_pose.orientation
            # Solo usar si el cuaternion no es identidad (tiene orientacion valida)
            if abs(q.x) > 1e-6 or abs(q.y) > 1e-6 or abs(q.z) > 1e-6:
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.goal_yaw = math.atan2(siny, cosy)
                self.get_logger().info(f"Orientacion objetivo: {math.degrees(self.goal_yaw):.1f} grados")
            else:
                self.goal_yaw = None
            
        self.current_target_index = 0
        self.last_target_index = 0
        if self._clock_initialized:
            self.last_advance_time = self.get_clock().now()
        self.ruta_completada = False
        self.pure_rotation_mode = True  # Iniciar cada nueva ruta girando hacia el objetivo
        self._check_initial_alignment = True  # One-shot: si ya hay alineacion, saltar pure_rotation
        self.rot_integral = 0.0 # Reset del integrador
        self.visual_search_mode = False
        self.visual_servo_mode = False
        self.vs_v_integral = 0.0
        self.vs_w_integral = 0.0
        self.vs_strafe_integral = 0.0
        self.terminal_approach_mode = False

    def cam_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
            
        self.img_w = cv_image.shape[1]
        
        # Dibujar markers detectados
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            if 1 in ids: # El cubo meta usa el ID 1 en sus caras laterales
                idx = list(ids.flatten()).index(1)
                esquinas = corners[idx][0]
                # Centroide del ArUco
                self.marker_cx = np.mean(esquinas[:, 0])
                # Lados para área y asimetría
                lado_sup = np.linalg.norm(esquinas[0] - esquinas[1])
                lado_der = np.linalg.norm(esquinas[1] - esquinas[2])
                lado_inf = np.linalg.norm(esquinas[2] - esquinas[3])
                lado_izq = np.linalg.norm(esquinas[3] - esquinas[0])
                self.marker_area = lado_sup * lado_der
                
                promedio = (lado_der + lado_izq) / 2.0
                self.marker_asymmetry = (lado_der - lado_izq) / promedio if promedio > 0 else 0.0
            else:
                self.marker_cx = None
                self.marker_area = None
                self.marker_asymmetry = None
        else:
            self.marker_cx = None
            self.marker_area = None
            self.marker_asymmetry = None
            
        # Publicar la imagen para rqt_image_view
        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Error al publicar imagen con ArUco: {e}")

    def odom_callback(self, msg):
        """Lectura de Odometría estándar Yahboom (msg.pose.pose)."""
        self._x_o = msg.pose.pose.position.x
        self._y_o = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        
        # Cálculo de Yaw (desde cuaternión)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._theta_o = math.atan2(siny_cosp, cosy_cosp)
        # Publicar TF dinámico de odom a base_footprint sin prefijos
        t = TransformStamped()
        t.header.stamp = msg.header.stamp if msg.header.stamp.sec > 0 else self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.dynamic_tf_broadcaster.sendTransform(t)

    def scan_callback(self, msg):
        obstaculo = False
        peligro_frontal = False
        rep_x = 0.0
        rep_y = 0.0
        
        self.min_dist_frontal = 5.0
        num_invalidos = 0
        
        self.fuerza_izq = 0.0
        self.fuerza_der = 0.0
        
        for i, r in enumerate(msg.ranges):
            # Ignorar distancias no válidas
            if r < 0.05 or math.isinf(r) or math.isnan(r):
                num_invalidos += 1
                continue

            angle = msg.angle_min + i * msg.angle_increment
            
            # Cono frontal original (30 grados totales: -15 a +15)
            es_frontal = abs(angle) < math.radians(20)
            umbral = self.umbral_frontal if es_frontal else self.umbral_lateral
            
            if es_frontal:
                self.min_dist_frontal = min(self.min_dist_frontal, r)

            if r < umbral:
                obstaculo = True
                if es_frontal:
                    peligro_frontal = True
                
                # Fuerza proporcional a la cercanía
                fuerza = 10.0 * ((umbral - r) / umbral)**2
                
                # Componente puramente repulsivo (empuja hacia atrás)
                rep_x += -fuerza * math.cos(angle)
                rep_y += -fuerza * math.sin(angle)
                
                # Para el Vórtice: saber de qué lado viene más fuerza
                if angle > 0:
                    self.fuerza_izq += fuerza
                else:
                    self.fuerza_der += fuerza
                
        if obstaculo:
            # Calcular fuerza tangencial (Vortex) perpendicular a la repulsión
            # Si el obstáculo está a la derecha, rodear por la izquierda (antihorario)
            if self.fuerza_der > self.fuerza_izq:
                tang_x = -rep_y
                tang_y = rep_x
            else:
                # Obstáculo a la izquierda, rodear por la derecha (horario)
                tang_x = rep_y
                tang_y = -rep_x
                
            # Combinación de repulsión (para no chocar) y tangencial (para deslizarse)
            peso_rep = 0.4
            peso_tang = 0.7
            
            self.repulsion_x = (rep_x * peso_rep) + (tang_x * peso_tang)
            self.repulsion_y = (rep_y * peso_rep) + (tang_y * peso_tang)
        else:
            self.repulsion_x = 0.0
            self.repulsion_y = 0.0
            
        self.obstaculo_cerca = obstaculo
        self.peligro_frontal = peligro_frontal
        
        if num_invalidos > len(msg.ranges) * 0.9:
            self.get_logger().error(
                "CRITICO: El LIDAR esta recibiendo casi todos los datos invalidos (inf/nan).",
                throttle_duration_sec=2.0,
            )

    def get_current_pose(self):
        # Intentar obtener la pose corregida por AMCL a traves de TF
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            self.first_aruco_received = True # Marca inicializado
            return x, y, theta
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            now = self.get_clock().now()
            if self.last_log_time is not None and (now - self.last_log_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().info(f"Esperando árbol TF (map -> base_footprint)... AMCL inicializando o desincronizado. Error: {str(e)}")
                self.last_log_time = now
            return None, None, None



    def get_target_point(self, x_c, y_c):
        if not self.path_points:
            return None
        # Buscar el punto mas cercano
        min_dist = float('inf')
        closest_idx = self.current_target_index
        # Empezamos a buscar desde el indice actual para no devolvemos en el camino
        for i in range(self.current_target_index, len(self.path_points)):
            px, py = self.path_points[i]
            dist = math.hypot(px - x_c, py - y_c)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        # Avanzar el indice con el lookahead
        target_idx = closest_idx
        for i in range(closest_idx, len(self.path_points)):
            px, py = self.path_points[i]
            dist = math.hypot(px - x_c, py - y_c)
            if dist >= self.lookahead_dist:
                target_idx = i
                break
        # Estrictamente tomamos el ultimo punto si ya estamos muy cerca del final
        if target_idx == closest_idx and target_idx == len(self.path_points) - 1:
            target_idx = len(self.path_points) - 1
        self.current_target_index = target_idx
        return self.path_points[target_idx]

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        # Guard: esperar a que el reloj de simulación sea válido (>0)
        # Esto evita que los timestamps se inicialicen en time=0 antes de que
        # Gazebo publique /clock, causando falsos timeouts en la primera ejecución.
        if not self._clock_initialized:
            now = self.get_clock().now()
            if now.nanoseconds == 0:
                return
            self.start_time = now
            self.last_advance_time = now
            self.last_log_time = now
            self._clock_initialized = True
            self.get_logger().info('Reloj de simulación válido. Control listo. Esperando ruta...')

        # Guard: no mover el carrito hasta que la alineacion mapa-SLAM este lista.
        # Mientras tanto, mantener velocidad cero y reiniciar el watchdog de
        # progreso para que no se dispare un replan injusto.
        if not self._alignment_ready:
            self.cmd_pub.publish(Twist())
            self.v_prev = 0.0
            self.w_prev = 0.0
            self.last_advance_time = self.get_clock().now()
            self.get_logger().info(
                'Esperando alineacion mapa-SLAM (/alignment_ready) antes de iniciar control...',
                throttle_duration_sec=3.0,
            )
            return

        now = self.get_clock().now()

        if not self.path_points:
            # Si pasaron mas de 3 segundos y no hay ruta, solicitarla
            if (now - self.start_time).nanoseconds / 1e9 > 3.0:
                self.get_logger().info('No se ha recibido la ruta inicial. Solicitando al planificador...', throttle_duration_sec=3.0)
                self.replan_pub.publish(Empty())
            return
            
        if self.ruta_completada:
            return
            
        x, y, theta = self.get_current_pose()
        if x is None:
            # FRENADO DE EMERGENCIA: Si perdemos la pose (lag de AMCL o desconexión TF),
            # no podemos seguir avanzando a ciegas. Frenar progresivamente.
            self.v_prev *= 0.5
            self.w_prev *= 0.5
            cmd = Twist()
            cmd.linear.x = float(self.v_prev)
            cmd.angular.z = float(self.w_prev)
            self.cmd_pub.publish(cmd)
            return
            
        # 1. Calcular punto de control desplazado (x_c, y_c)
        x_c = x + self.h * math.cos(theta)
        y_c = y + self.h * math.sin(theta)
        # 2. Obtener punto objetivo de la ruta
        target = self.get_target_point(x_c, y_c)
        if not target:
            return
        x_d, y_d = target
        # Distancia del CENTRO del robot al ultimo punto del path (no del punto desplazado x_c)
        dist_to_final = math.hypot(self.path_points[-1][0] - x, self.path_points[-1][1] - y)

        # Status log throttled a 5s: modo actual, waypoint, dist a meta, dist frontal LiDAR
        if self.visual_servo_mode:
            mode_str = "VISUAL_SERVO"
        elif self.visual_search_mode:
            mode_str = "VISUAL_SEARCH"
        elif self.terminal_approach_mode:
            mode_str = "TERMINAL"
        elif self.pure_rotation_mode:
            mode_str = "PURE_ROT"
        else:
            mode_str = "KD"
        self.get_logger().info(
            f"[STATUS] mode={mode_str} wpt={self.current_target_index}/{len(self.path_points)} "
            f"dist_final={dist_to_final:.2f}m min_front={self.min_dist_frontal:.2f}m",
            throttle_duration_sec=5.0,
        )

        # ---- Llegada a la meta: Kelly & Diaz -> Aproximacion terminal -> Busqueda Visual ----
        if not self.visual_search_mode and not self.visual_servo_mode:
            if not self.terminal_approach_mode and dist_to_final < self.terminal_threshold:
                self.terminal_approach_mode = True
                self.v_prev = 0.0
                self.w_prev = 0.0
                self.get_logger().info(
                    f"Fin de Kelly & Diaz (dist={dist_to_final:.3f}m). Aproximacion terminal go-to-point..."
                )
            if self.terminal_approach_mode and dist_to_final < self.terminal_arrival:
                self.terminal_approach_mode = False
                self.v_prev = 0.0
                self.w_prev = 0.0
                if self.disable_visual_modes:
                    self.ruta_completada = True
                    self.stop_robot()
                    self.get_logger().info(
                        f"Goal alcanzado (dist={dist_to_final:.3f}m). Modos visuales deshabilitados, deteniendo."
                    )
                    return
                self.visual_search_mode = True
                self.get_logger().info(
                    f"Aproximacion terminal completa (dist={dist_to_final:.3f}m). Iniciando busqueda visual..."
                )

        if self.visual_search_mode:
            cmd = Twist()
            cmd.linear.x = 0.0
            
            if self.marker_cx is not None:
                self.get_logger().info("¡ArUco Encontrado! Pasando a modo Visual Servoing.")
                self.visual_search_mode = False
                self.visual_servo_mode = True
                self.vs_v_integral = 0.0
                self.vs_w_integral = 0.0
                self.vs_strafe_integral = 0.0
            else:
                # Rotar sobre su eje hasta encontrarlo
                cmd.angular.z = 0.6
                self.cmd_pub.publish(cmd)
            return

        if self.visual_servo_mode:
            cmd = Twist()
            if self.marker_cx is None:
                # Lo perdimos de vista, volver a buscar
                self.visual_search_mode = True
                self.visual_servo_mode = False
                return
                
            dt = 0.05 # Lazo a 20Hz
            
            # 1. Rotación: centrar marker en la imagen (Agresivo para mantener el objetivo a la vista)
            err_x = (self.img_w / 2.0) - self.marker_cx
            self.vs_w_integral += err_x * dt
            self.vs_w_integral = max(-100.0, min(100.0, self.vs_w_integral)) # anti-windup
            
            # Aumentamos aún más la ganancia Proporcional e Integral (Rotación muy agresiva)
            w_cmd = (err_x * 0.010) + (self.vs_w_integral * 0.004)
            
            # Aumentamos la velocidad máxima de rotación a 1.2
            cmd.angular.z = max(min(w_cmd, 1.2), -1.2)
            
            # 2. Strafe lateral: corregir asimetría para lograr perpendicularidad
            # Asimetría > 0 -> lado der más grande -> robot a la derecha -> strafe izq (+y)
            self.vs_strafe_integral += self.marker_asymmetry * dt
            self.vs_strafe_integral = max(-1.0, min(1.0, self.vs_strafe_integral)) # anti-windup
            strafe_cmd = (self.marker_asymmetry * 0.5) + (self.vs_strafe_integral * 0.1)
            cmd.linear.y = max(min(strafe_cmd, 0.15), -0.15)
            
            # 3. Avance: controlar distancia con LiDAR (más robusto que el área en píxeles)
            distancia_objetivo = 0.3# Queremos estacionarnos a 25 cm del cubo
            dist_err = self.min_dist_frontal - distancia_objetivo
            
            self.vs_v_integral += dist_err * dt
            self.vs_v_integral = max(-0.5, min(0.5, self.vs_v_integral)) # anti-windup
            v_cmd = (dist_err * 0.4) + (self.vs_v_integral * 0.15)
            
            # Tolerancia de paro para avance
            if abs(dist_err) > 0.05:
                # FASE 1 y 2: Primero centrar, luego avanzar. 
                # Solo permitimos avance si el ArUco está relativamente centrado frente a la cámara.
                if abs(err_x) < 50 and abs(self.marker_asymmetry) < 0.20:
                    cmd.linear.x = max(min(v_cmd, 0.12), -0.12)
                else:
                    cmd.linear.x = 0.0
            else:
                cmd.linear.x = 0.0
            
            # 4. Condición de Paro: Distancia correcta (LiDAR), marker MUY centrado y asimetría mínima
            if abs(dist_err) < 0.05 and abs(err_x) < 15 and abs(self.marker_asymmetry) < 0.05:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.get_logger().info(f"Parqueo completado. Dist: {self.min_dist_frontal:.2f}m, Asimetria: {self.marker_asymmetry:.3f}")
                self.ruta_completada = True
                return

            self.cmd_pub.publish(cmd)
            return

        if self.terminal_approach_mode:
            x_goal_w = self.path_points[-1][0]
            y_goal_w = self.path_points[-1][1]
            e_x_world = x_goal_w - x
            e_y_world = y_goal_w - y
            e_x_local = e_x_world * math.cos(theta) + e_y_world * math.sin(theta)
            e_y_local = -e_x_world * math.sin(theta) + e_y_world * math.cos(theta)

            cmd = Twist()
            v_x = self.k_v_terminal * e_x_local
            v_y = self.k_v_terminal * e_y_local
            
            # Velocidad mínima para no atascarse por fricción
            if 0 < abs(v_x) < 0.05: v_x = 0.05 if v_x > 0 else -0.05
            if 0 < abs(v_y) < 0.05: v_y = 0.05 if v_y > 0 else -0.05
            
            cmd.linear.x = max(min(v_x, self.v_max_terminal), -self.v_max_terminal)
            cmd.linear.y = max(min(v_y, self.v_max_terminal), -self.v_max_terminal)

            # En modo terminal desactivamos la rotación para no perder de vista el ArUco
            # (dejamos que el visual servoing se encargue de la orientación final).
            # Además, rotar con ruedas mecanum en Gazebo causa deriva hacia adelante.
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)
            self.v_prev = cmd.linear.x
            self.w_prev = cmd.angular.z
            return

        # Watchdog: Si no avanzamos de punto en 4 segundos, replanificar
        # EXCEPCION: No replanificar si ya estamos cerca de la meta final
        near_final_goal = (self.current_target_index >= len(self.path_points) - 2 and dist_to_final < self.terminal_threshold + 0.05)
        now = self.get_clock().now()

        if self.current_target_index > self.last_target_index:
            self.last_target_index = self.current_target_index
            self.last_advance_time = now
        else:
            elapsed = (now - self.last_advance_time).nanoseconds / 1e9
            # Limite de watchdog dinamico: 10s en rotacion pura, 5s en seguimiento normal
            timeout = 10.0 if self.pure_rotation_mode else 5.0
            if elapsed > timeout and not near_final_goal:
                self.get_logger().warn(f"Bloqueo detectado! No se avanza de waypoint en {timeout}s. Solicitando replanificacion...")
                self.replan_pub.publish(Empty())
                self.last_advance_time = now # Reset para no saturar mientras llega la nueva ruta

        # 5. Calcular errores
        e_x = x_d - x_c
        e_y = y_d - y_c
        
        # Velocidad Dinamica basada en Lidar Frontal
        # Aumentamos la velocidad hasta 0.7 m/s si hay espacio libre (>0.5m)
        v_dinamica = self.v_max
        if self.min_dist_frontal > 0.5:
            # Rango: 0.5m -> factor 0.0, 1.0m -> factor 1.0
            factor = min(1.0, (self.min_dist_frontal - 0.5) / 0.5)
            v_dinamica = self.v_max + factor * (0.7 - self.v_max)
            
        # Para hacer el seguimiento más dinamico, calculamos una velocidad de feedforward hacia el objetivo
        # Si fueramos un tracker puro en el tiempo, estas serían las derivadas de la trayectoria, 
        # pero para seguimiento espacial de puntos (Path Tracking), las definimos apuntando al objetivo.
        dist_to_target = math.hypot(e_x, e_y)
        if dist_to_target > 0:
            v_ref_x = v_dinamica * (e_x / dist_to_target)
            v_ref_y = v_dinamica * (e_y / dist_to_target)
        else:
            v_ref_x = 0.0
            v_ref_y = 0.0

        # Velocidades de control en el mundo (u1, u2) (Atracción con prioridad dinámica)
        # FRENADO PREVENTIVO: Si estamos a menos de 50cm, reducimos k_att gradualmente hasta 0 en los 20cm (punto ciego)
        if self.min_dist_frontal < self.umbral_frontal:
            k_att = (self.min_dist_frontal - 0.20) / (self.umbral_frontal - 0.20)
            k_att = max(0.0, min(1.0, k_att))
        else:
            k_att = 1.0
            
        u1 = k_att * (v_ref_x + self.k_px * e_x)
        u2 = k_att * (v_ref_y + self.k_py * e_y)

        # Capa de Reactividad (Evasión Dinámica Local convertida a Mundo)
        if self.obstaculo_cerca:
            # Rotar los vectores locales del carrito hacia el mundo(mapa)
            rep_mundo_x = (self.repulsion_x * math.cos(theta) - self.repulsion_y * math.sin(theta))
            rep_mundo_y = (self.repulsion_x * math.sin(theta) + self.repulsion_y * math.cos(theta))
            
            # LIMITAR MAGNITUD (Capping): La repulsión no puede ser infinita
            # Reducimos max_rep para evitar que el robot se quede bloqueado en pasillos estrechos
            rep_mag = math.hypot(rep_mundo_x, rep_mundo_y)
            max_rep = 0.45  # Aumentado para vencer mejor la inercia en esquinas (antes 0.25)
            if rep_mag > max_rep:
                rep_mundo_x = (rep_mundo_x / rep_mag) * max_rep
                rep_mundo_y = (rep_mundo_y / rep_mag) * max_rep

            u1 += rep_mundo_x
            u2 += rep_mundo_y
        # 4. Transformar a velocidades del robot diferencial (v, w)
        v = u1 * math.cos(theta) + u2 * math.sin(theta)
        w = (-u1 * math.sin(theta) + u2 * math.cos(theta)) / self.h
        
        # SOBRESCRITURA DE CONTROL: Si estamos en modo Rotación Pura (inicio de ruta)
        if self.pure_rotation_mode:
            # Apuntar exactamente a target_x, target_y usando el centro del robot
            theta_d = math.atan2(y_d - y, x_d - x)
            e_theta = theta_d - theta
            # Normalizar ángulo
            while e_theta > math.pi: e_theta -= 2.0 * math.pi
            while e_theta < -math.pi: e_theta += 2.0 * math.pi

            # Chequeo one-shot al recibir la ruta: si ya estamos suficientemente
            # alineados (~17 deg), saltar rotacion pura y arrancar con Kelly & Diaz.
            if self._check_initial_alignment:
                self._check_initial_alignment = False
                if abs(e_theta) < 0.30:
                    self.pure_rotation_mode = False
                    self.get_logger().info(
                        f"Yaw inicial OK ({math.degrees(e_theta):.1f} deg), saltando rotacion pura."
                    )

            # Si seguimos en rotacion pura, control mixto:
            # rotar mientras se traslada despacio para arrancar de inmediato.
            if self.pure_rotation_mode and abs(e_theta) > 0.15:
                # Traslacion parcial decreciente con el error angular:
                # |e|=0 → v_arranque, |e|>=0.5 rad → v=0
                v_arranque = 0.08
                v = v_arranque * max(0.0, 1.0 - abs(e_theta) / 0.5)
                
                # Control PI de rotacion (friccion en hardware real)
                dt = 0.05 # Lazo a ~20Hz
                self.rot_integral += e_theta * dt
                self.rot_integral = max(-1.0, min(1.0, self.rot_integral)) # Limite anti-windup
                
                w = 1.0 * e_theta + 0.2 * self.rot_integral
                if abs(w) < 0.35:
                    w = 0.35 if w > 0 else -0.35
            elif self.pure_rotation_mode:
                self.pure_rotation_mode = False # Termina la rotación, sigue Kelly & Diaz
                self.rot_integral = 0.0
        

        # 5. Saturación de control
        v = max(min(v, v_dinamica), -v_dinamica)
        w = max(min(w, self.w_max), -self.w_max)
        # 6. SUAVIZADO: Más alto = más reactivo (0.1 = muy lento, 1.0 = instantáneo)
        alpha_v = 0.8 
        alpha_w = 0.8 
        self.v_prev = (alpha_v * v) + (1.0 - alpha_v) * self.v_prev
        self.w_prev = (alpha_w * w) + (1.0 - alpha_w) * self.w_prev

        # 7. Publicar comandos
        cmd = Twist()
        cmd.linear.x = float(self.v_prev)
        cmd.angular.z = float(self.w_prev)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlTrayectoria()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
