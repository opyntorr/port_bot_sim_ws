#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformBroadcaster
import tf2_ros
import tf_transformations
import math
import numpy as np

class ControlTrayectoria(Node):
    def __init__(self):
        super().__init__('control_trayectoria')
        
        # El parametro use_sim_time se gestiona automaticamente por ROS 2 si se pasa en el comando.
        
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
        self.final_rotation_mode = False  # Fase de alineamiento final en la meta
        self.goal_yaw = None  # Orientacion objetivo (se extrae del ultimo punto del Path)
        
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
        
        # Estado del LiDAR y evasión
        self.repulsion_x = 0.0
        self.repulsion_y = 0.0
        self.obstaculo_cerca = False
        self.peligro_frontal = False
        self.min_dist_frontal = 1.0
        self.umbral_frontal = 0.35 
        self.umbral_lateral = 0.25 
        
        self.last_log_time = None  # Se inicializa cuando el reloj es válido

        
        qos_scan = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.scan_sub = self.create_subscription(LaserScan, '/scan_filtered', self.scan_callback, qos_scan)
        
        # Bucle de control a 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        
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
        self.ruta_completada = False
        self.pure_rotation_mode = True  # Iniciar cada nueva ruta girando hacia el objetivo
        self.final_rotation_mode = False

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
        
        self.min_dist_frontal = 1.0
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
        
        # LOG DE DIAGNÓSTICO CADA SEGUNDO
        now = self.get_clock().now()
        if self.last_log_time is not None and (now - self.last_log_time).nanoseconds / 1e9 > 1.0:
            if not obstaculo:
                 self.get_logger().info(f"LIDAR: Camino despejado. Min Frontal: {self.min_dist_frontal:.2f}m")
            else:
                 self.get_logger().warn(f"LIDAR: ¡OBSTÁCULO! Dist: {self.min_dist_frontal:.2f}m | Fuerza Rep: [{self.repulsion_x:.2f}, {self.repulsion_y:.2f}]")
            
            if num_invalidos > len(msg.ranges) * 0.9:
                self.get_logger().error("CRITICO: El LIDAR esta recibiendo casi todos los datos invalidos (inf/nan).")

    def get_current_pose(self):
        # 1. Intentar obtener la pose corregida por AMCL a traves de TF
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            self.first_aruco_received = True # Marca inicializado
            return x, y, theta
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        # 2. Obtener la odometria del callback directo (FALLBACK)
        x_o = self._x_o
        y_o = self._y_o
        theta_o = self._theta_o

        # 3. Intentar obtener la pose absoluta del aruco (FALLBACK INICIAL)
        x_a_raw, y_a_raw, theta_a_raw = None, None, None
        try:
            trans_aruco = self.tf_buffer.lookup_transform('map', 'carrito_aruco', rclpy.time.Time())
            stamp = trans_aruco.header.stamp
            current_aruco_timestamp = (stamp.sec, stamp.nanosec)
            
            x_a_raw = trans_aruco.transform.translation.x
            y_a_raw = trans_aruco.transform.translation.y
            q_a_raw = trans_aruco.transform.rotation
            euler_a_raw = tf_transformations.euler_from_quaternion([q_a_raw.x, q_a_raw.y, q_a_raw.z, q_a_raw.w])
            theta_a_raw = euler_a_raw[2]
            
            if self.last_aruco_timestamp != current_aruco_timestamp:
                self.last_aruco_timestamp = current_aruco_timestamp
                
                if x_o is not None and not self.first_aruco_received:
                    theta_a_map = theta_a_raw
                    self.odom_offset_theta = math.atan2(math.sin(theta_a_map - theta_o), math.cos(theta_a_map - theta_o))
                    self.odom_offset_x = x_a_raw - (x_o * math.cos(self.odom_offset_theta) - y_o * math.sin(self.odom_offset_theta))
                    self.odom_offset_y = y_a_raw - (x_o * math.sin(self.odom_offset_theta) + y_o * math.cos(self.odom_offset_theta))
                    
                    self.first_aruco_received = True
                    self.get_logger().info("Snapshot ArUco registrado. AMCL se encargará de map->odom.")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        # 4. Calcular la posicion final estimada combinada (FALLBACK)
        if x_o is not None:
            if self.first_aruco_received:
                x_curr = self.odom_offset_x + x_o * math.cos(self.odom_offset_theta) - y_o * math.sin(self.odom_offset_theta)
                y_curr = self.odom_offset_y + x_o * math.sin(self.odom_offset_theta) + y_o * math.cos(self.odom_offset_theta)
                theta_curr = self.odom_offset_theta + theta_o
                theta_curr = math.atan2(math.sin(theta_curr), math.cos(theta_curr))
                return x_curr, y_curr, theta_curr
            else:
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                if elapsed > 4.0:
                    self.get_logger().warn("No se detecta ArUco. Iniciando por Odometria cruda.")
                    self.odom_offset_x = 0.0
                    self.odom_offset_y = 0.0
                    self.odom_offset_theta = 0.0
                    self.first_aruco_received = True
                    return x_o, y_o, theta_o

        now = self.get_clock().now()
        if self.last_log_time is not None and (now - self.last_log_time).nanoseconds / 1e9 > 5.0:
            if x_o is None:
                self.get_logger().info("Esperando datos de odometría en /odom...")
            else:
                self.get_logger().info("Odometría OK. Esperando snapshot de ArUco o AMCL (4s timeout)...")
            self.last_log_time = now

        if x_a_raw is not None:
            return x_a_raw, y_a_raw, theta_a_raw

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
            self.get_logger().info('Reloj de simulación válido. Control listo.')
        
        if not self.path_points or self.ruta_completada:
            return
        x, y, theta = self.get_current_pose()
        if x is None:
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

        # ---- Llegada a la meta: detectar por distancia, no por indice ----
        # Threshold de posicion estricto. Una vez detectado, NO se vuelve a mover linealmente.
        pos_threshold = 0.05  # 5 cm de tolerancia en posicion final
        if not self.final_rotation_mode and dist_to_final < pos_threshold:
            self.final_rotation_mode = True
            # Reset de los filtros para que el suavizado no arrastre velocidad lineal previa
            self.v_prev = 0.0
            self.w_prev = 0.0
            if self.goal_yaw is not None:
                self.get_logger().info(
                    f"Posicion alcanzada (dist={dist_to_final:.3f}m). "
                    f"Alineando a {math.degrees(self.goal_yaw):.1f} grados..."
                )
            else:
                self.get_logger().info(f"Posicion alcanzada (dist={dist_to_final:.3f}m). Sin orientacion objetivo, deteniendo.")

        if self.final_rotation_mode:
            # Solo rotacion: linear.x SIEMPRE 0, no volver a moverse aunque la pose derive un poco
            cmd = Twist()
            cmd.linear.x = 0.0

            if self.goal_yaw is None:
                # No hay orientacion objetivo: parar y terminar
                self.cmd_pub.publish(cmd)
                self.get_logger().info("Meta alcanzada con precision.")
                self.ruta_completada = True
                return

            e_theta = self.goal_yaw - theta
            while e_theta > math.pi: e_theta -= 2.0 * math.pi
            while e_theta < -math.pi: e_theta += 2.0 * math.pi

            if abs(e_theta) > 0.05:  # ~3 grados
                w_cmd = 1.2 * e_theta
                if abs(w_cmd) < 0.3:
                    w_cmd = 0.3 if e_theta > 0 else -0.3
                cmd.angular.z = max(min(w_cmd, self.w_max), -self.w_max)
                self.cmd_pub.publish(cmd)
                return
            else:
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.get_logger().info("Orientacion final alcanzada. Meta completada.")
                self.ruta_completada = True
                return
            
        # 3. Log de progreso cada 2 segundos con Telemetría de Velocidad
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 > 2.0:
            percent = (self.current_target_index + 1) / len(self.path_points) * 100
            # Incluimos V y W actuales para ver si el robot se está intentando mover
            self.get_logger().info(f"Seguimiento: {self.current_target_index + 1}/{len(self.path_points)} ({percent:.1f}%) | V: {self.v_prev:.2f} W: {self.w_prev:.2f}")
            self.last_log_time = now

        # 4. Watchdog: Si no avanzamos de punto en 4 segundos, replanificar
        # EXCEPCION: No replanificar si ya estamos cerca de la meta final
        near_final_goal = (self.current_target_index >= len(self.path_points) - 2 and dist_to_final < 0.25)
        now = self.get_clock().now()
        
        # LOG DE PULSO VITAL: Angulo actual crudo (para ver si se mueve)
        if (now - self.last_log_time).nanoseconds / 1e9 > 1.0:
             self.get_logger().info(f"[DIAGNOSTICO] ANGULO CRUDO ODOM: {math.degrees(theta):.2f} | Dist meta: {dist_to_final:.3f}m")
        
        if self.current_target_index > self.last_target_index:
            self.last_target_index = self.current_target_index
            self.last_advance_time = now
        else:
            elapsed = (now - self.last_advance_time).nanoseconds / 1e9
            if elapsed > 4.0 and not near_final_goal:
                self.get_logger().warn("Bloqueo detectado! No se avanza de waypoint en 4s. Solicitando replanificacion...")
                self.replan_pub.publish(Empty())
                self.last_advance_time = now # Reset para no saturar mientras llega la nueva ruta

        # 5. Calcular errores
        e_x = x_d - x_c
        e_y = y_d - y_c
        # Para hacer el seguimiento más dinamico, calculamos una velocidad de feedforward hacia el objetivo
        # Si fueramos un tracker puro en el tiempo, estas serían las derivadas de la trayectoria, 
        # pero para seguimiento espacial de puntos (Path Tracking), las definimos apuntando al objetivo.
        dist_to_target = math.hypot(e_x, e_y)
        if dist_to_target > 0:
            v_ref_x = self.v_max * (e_x / dist_to_target)
            v_ref_y = self.v_max * (e_y / dist_to_target)
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
            
            # Aumentar ventana de tolerancia a ~8.5 grados para evitar saltar sobre la meta por el drift
            if abs(e_theta) > 0.15:
                v = 0.0
                # Control P más suave para no sobre-reaccionar y vencer fricción
                w = 1.0 * e_theta
                if abs(w) < 0.35:
                    w = 0.35 if w > 0 else -0.35
            else:
                self.pure_rotation_mode = False # Termina la rotación, sigue Kelly & Diaz
        

        # 5. Saturación de control
        v = max(min(v, self.v_max), -self.v_max)
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
