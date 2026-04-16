#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
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
        
        # Suscriptor directo de odometria (mas robusto que TF lookup en Gazebo Sim)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self._x_o = None
        self._y_o = None
        self._theta_o = None
        
        # Parametros del controlador de Kelly & Diaz (Sintonizados para fidelidad)
        self.h = 0.08          # Desplazamiento del punto de interes
        self.k_px = 2.0        # Ganancia proporcional (ligeramente menor para suavidad)
        self.k_py = 2.0        
        self.v_max = 0.2      # Velocidad maxima permitida
        self.w_max = 1.2       # Velocidad angular maxima permitida

        # Variables para suavizado (Filtro Pasa-Baja)
        self.v_prev = 0.0
        self.w_prev = 0.0


        
        self.lookahead_dist = 0.1 # Distancia para buscar el siguiente punto objetivo
        
        self.current_target_index = 0
        self.ruta_completada = False
        
        # Variables para fusion de odometria
        self.odom_offset_x = 0.0
        self.odom_offset_y = 0.0
        self.odom_offset_theta = 0.0
        self.last_aruco_timestamp = None
        self.first_aruco_received = False
        
        # Variables de evasion dinamica y lidar
        self.repulsion_x = 0.0
        self.repulsion_y = 0.0
        self.obstaculo_cerca = False
        self.distancia_seguridad = 0.15 # Reducido para pasillos estrechos

        
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
            
        self.current_target_index = 0
        self.ruta_completada = False

    def odom_callback(self, msg):
        """Lectura directa de odometria por topico (bypass de TF)."""
        self._x_o = msg.pose.pose.position.x
        self._y_o = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._theta_o = euler[2]

    def scan_callback(self, msg):
        rep_x = 0.0
        rep_y = 0.0
        obstaculo = False
        
        for i, r in enumerate(msg.ranges):
            # Ignorar distancias no válidas
            if r < 0.05 or math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            
            # UMBRAL DINÁMICO: 
            # 0.5 metros al frente (cono de 30 grados: -15 a +15)
            # 0.2 metros a los lados (el resto de los 110 grados)
            umbral = 0.35 if abs(angle) < math.radians(15) else 0.15
            
            if r < umbral:
                obstaculo = True
                
                # Fuerza asintótica proporcional al umbral de la zona
                fuerza = - ((umbral - r) / umbral)
                
                # Sumatoria local en base_footprint
                rep_x += fuerza * math.cos(angle)
                rep_y += fuerza * math.sin(angle)
                
        self.obstaculo_cerca = obstaculo
        if obstaculo:
            self.repulsion_x = rep_x
            self.repulsion_y = rep_y
        else:
            self.repulsion_x = 0.0
            self.repulsion_y = 0.0

    def get_current_pose(self):
        # 1. Obtener la odometria del callback directo
        x_o = self._x_o
        y_o = self._y_o
        theta_o = self._theta_o

        # 2. Intentar obtener la pose absoluta del aruco
        x_a_raw, y_a_raw, theta_a_raw = None, None, None
        try:
            trans_aruco = self.tf_buffer.lookup_transform('map', 'carrito_aruco', rclpy.time.Time())
            stamp = trans_aruco.header.stamp
            current_aruco_timestamp = (stamp.sec, stamp.nanosec)
            
            # Guardar pose cruda del ArUco como fallback
            x_a_raw = trans_aruco.transform.translation.x
            y_a_raw = trans_aruco.transform.translation.y
            q_a_raw = trans_aruco.transform.rotation
            euler_a_raw = tf_transformations.euler_from_quaternion([q_a_raw.x, q_a_raw.y, q_a_raw.z, q_a_raw.w])
            theta_a_raw = euler_a_raw[2]
            theta_a_raw -= - math.pi / 2
            
            # Solo actualizar offset si el tiempo avanzo (nueva lectura de camara)
            if self.last_aruco_timestamp != current_aruco_timestamp:
                self.last_aruco_timestamp = current_aruco_timestamp
                
                if x_o is not None:
                    # Calcular el target offset (transformacion de map a odom)
                    target_offset_theta = math.atan2(math.sin(theta_a_raw - theta_o), math.cos(theta_a_raw - theta_o))
                    target_offset_x = x_a_raw - (x_o * math.cos(target_offset_theta) - y_o * math.sin(target_offset_theta))
                    target_offset_y = x_a_raw - (x_o * math.sin(target_offset_theta) + y_o * math.cos(target_offset_theta))
                    
                    if not self.first_aruco_received:
                        self.odom_offset_x = target_offset_x
                        self.odom_offset_y = target_offset_y
                        self.odom_offset_theta = target_offset_theta
                        self.first_aruco_received = True
                        self.get_logger().info("Odometria inicializada con ArUco.")
                    else:
                        # FUSION CONTINUA: Filtro pasa-baja (alpha=0.1)
                        # 90% odometria (fluidez) + 10% ArUco (precision global)
                        alpha = 0.1
                        self.odom_offset_x = (1.0 - alpha) * self.odom_offset_x + alpha * target_offset_x
                        self.odom_offset_y = (1.0 - alpha) * self.odom_offset_y + alpha * target_offset_y
                        
                        diff_theta = math.atan2(math.sin(target_offset_theta - self.odom_offset_theta), 
                                                math.cos(target_offset_theta - self.odom_offset_theta))
                        self.odom_offset_theta += alpha * diff_theta
                        self.odom_offset_theta = math.atan2(math.sin(self.odom_offset_theta), math.cos(self.odom_offset_theta))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        # 3. Calcular la posicion final estimada combinada
        if x_o is not None and self.first_aruco_received:
            x_curr = self.odom_offset_x + x_o * math.cos(self.odom_offset_theta) - y_o * math.sin(self.odom_offset_theta)
            y_curr = self.odom_offset_y + x_o * math.sin(self.odom_offset_theta) + y_o * math.cos(self.odom_offset_theta)
            theta_curr = self.odom_offset_theta + theta_o
            theta_curr = math.atan2(math.sin(theta_curr), math.cos(theta_curr))
            return x_curr, y_curr, theta_curr

        # FALLBACK: Si no hay odom aun, usar ArUco crudo para arrancar
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
        # Evaluar si llegamos al destino final
        dist_to_final = math.hypot(self.path_points[-1][0] - x_c, self.path_points[-1][1] - y_c)
        if self.current_target_index == len(self.path_points) - 1 and dist_to_final < 0.15:
            self.get_logger().info("¡Llegamos a la meta!")
            self.stop_robot()
            self.ruta_completada = True
            return

        # 3. Calcular errores
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

        # Velocidades de control en el mundo (u1, u2) (Atraccion pura)
        u1 = v_ref_x + self.k_px * e_x
        u2 = v_ref_y + self.k_py * e_y
        # Capa de Reactividad (Evasion Dinamica Local convertida a Mundo)
        if self.obstaculo_cerca:
            k_rep = 1.0 # Multiplicador de fuerza repulsiva
            # Rotar los vectores locales del carrito hacia el mundo(mapa)
            rep_mundo_x = (self.repulsion_x * math.cos(theta) - self.repulsion_y * math.sin(theta)) * k_rep
            rep_mundo_y = (self.repulsion_x * math.sin(theta) + self.repulsion_y * math.cos(theta)) * k_rep
            u1 += rep_mundo_x
            u2 += rep_mundo_y
        # 4. Transformar a velocidades del robot diferencial (v, w)
        v = u1 * math.cos(theta) + u2 * math.sin(theta)
        w = (-u1 * math.sin(theta) + u2 * math.cos(theta)) / self.h
        # 5. Saturación de control
        v = max(min(v, self.v_max), -self.v_max)
        w = max(min(w, self.w_max), -self.w_max)
        # 6. SUAVIZADO: Filtro Pasa-Baja (0.1 = muy suave, 1.0 = sin filtro)
        alpha_v = 0.2 # Suavizado para velocidad lineal
        alpha_w = 0.3 # Suavizado para velocidad angular
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
