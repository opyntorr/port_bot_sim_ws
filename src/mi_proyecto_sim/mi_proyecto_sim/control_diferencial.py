#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Bool
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf2_ros
import tf_transformations
import math
import numpy as np

class ControlDiferencial(Node):
    def __init__(self):
        super().__init__('control_diferencial')

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.path_sub = self.create_subscription(Path, '/rrt_path', self.path_callback, qos_profile)
        self.path_points = []
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)
        
        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_pose)
        
        # Kelly & Diaz Parameters
        self.h = 0.30   # punto de control K&D; subido de 0.25 (era del Rosmaster X3, mas chico) al tamano del JetAuto: giros mas suaves
        self.k_px = 1.5
        self.k_py = 1.5        
        self.v_max = 0.20      # m/s (coherente con cap chassis 0.25; conservador para SLAM)
        self.w_max = 0.5       # rad/s (giro suave; coherente con cap chassis 0.5)

        self.v_prev = 0.0
        self.w_prev = 0.0
        self.lookahead_dist = 0.15   # subido de 0.12; reduce serpenteo sin cortar curvas
        
        self.current_target_index = 0
        self.ruta_completada = False
        self.pure_rotation_mode = False
        self._check_initial_alignment = False
        self.rot_integral = 0.0
        
        self.replan_pub = self.create_publisher(Empty, '/replan_request', 10)
        self.last_advance_time = None
        self.last_target_index = 0
        self.start_time = None
        self._clock_initialized = False

        self._alignment_ready = False
        qos_latch = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.alignment_sub = self.create_subscription(Bool, '/alignment_ready', self._alignment_callback, qos_latch)
        
        # LiDAR Evasion State
        self.repulsion_x = 0.0
        self.repulsion_y = 0.0
        self.obstaculo_cerca = False
        self.min_dist_frontal = 1.5
        self.umbral_frontal = 0.55 
        self.umbral_lateral = 0.40 
        self.fuerza_izq = 0.0
        self.fuerza_der = 0.0
        
        qos_scan = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        # scan_topic: por defecto /scan (sim sin filtro). El robot real pasa /scan_filtered.
        scan_topic = self.declare_parameter('scan_topic', '/scan').get_parameter_value().string_value
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_scan)
        self.last_log_time = None

        self.timer = self.create_timer(0.05, self.control_loop)

    def _alignment_callback(self, msg):
        if msg.data and not self._alignment_ready:
            self.get_logger().info('Alineacion mapa-SLAM confirmada. Habilitando control diferencial.')
        self._alignment_ready = bool(msg.data)

    def path_callback(self, msg):
        nueva = []
        for pose_stamped in msg.poses:
            nueva.append((pose_stamped.pose.position.x, pose_stamped.pose.position.y))
        if not nueva:
            return

        tenia_ruta = bool(self.path_points)
        self.path_points = nueva

        # NO reiniciar el progreso a 0 si ya ibamos siguiendo una ruta: el RRT replanifica
        # cada ~1s (para obstaculos dinamicos) y resetear a 0 cada vez TRABA al robot
        # (avanza, llega ruta nueva, vuelve al inicio, tiembla). En cambio, retomamos desde
        # el waypoint mas cercano a la pose actual -> el seguimiento continua sin saltos.
        if tenia_ruta and self._clock_initialized:
            x, y, theta = self.get_current_pose()
            if x is not None:
                xc = x + self.h * math.cos(theta)
                yc = y + self.h * math.sin(theta)
                idx_cercano = min(
                    range(len(nueva)),
                    key=lambda i: (nueva[i][0]-xc)**2 + (nueva[i][1]-yc)**2)
                self.current_target_index = idx_cercano
                self.last_target_index = idx_cercano
            else:
                self.current_target_index = 0
                self.last_target_index = 0
        else:
            # primera ruta: empezar desde el principio
            self.get_logger().info("Ruta recibida. Iniciando seguimiento diferencial...")
            self.current_target_index = 0
            self.last_target_index = 0
            self.pure_rotation_mode = True

        if self._clock_initialized:
            self.last_advance_time = self.get_clock().now()
        self.ruta_completada = False
        self._check_initial_alignment = True
        self.rot_integral = 0.0

    def odom_callback(self, msg):
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
        rep_x = 0.0
        rep_y = 0.0
        self.min_dist_frontal = 5.0
        self.fuerza_izq = 0.0
        self.fuerza_der = 0.0
        
        for i, r in enumerate(msg.ranges):
            if r < 0.05 or math.isinf(r) or math.isnan(r): continue

            angle = msg.angle_min + i * msg.angle_increment
            es_frontal = abs(angle) < math.radians(20)
            umbral = self.umbral_frontal if es_frontal else self.umbral_lateral
            
            if es_frontal:
                self.min_dist_frontal = min(self.min_dist_frontal, r)

            if r < umbral:
                obstaculo = True
                fuerza = 10.0 * ((umbral - r) / umbral)**2
                rep_x += -fuerza * math.cos(angle)
                rep_y += -fuerza * math.sin(angle)
                
                if angle > 0: self.fuerza_izq += fuerza
                else: self.fuerza_der += fuerza
                
        if obstaculo:
            if self.fuerza_der > self.fuerza_izq:
                tang_x, tang_y = -rep_y, rep_x
            else:
                tang_x, tang_y = rep_y, -rep_x
                
            self.repulsion_x = (rep_x * 0.4) + (tang_x * 0.7)
            self.repulsion_y = (rep_y * 0.4) + (tang_y * 0.7)
        else:
            self.repulsion_x = 0.0
            self.repulsion_y = 0.0
        self.obstaculo_cerca = obstaculo

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            return x, y, theta
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return None, None, None

    def get_target_point(self, x_c, y_c):
        if not self.path_points: return None
        min_dist = float('inf')
        closest_idx = self.current_target_index
        for i in range(self.current_target_index, len(self.path_points)):
            px, py = self.path_points[i]
            dist = math.hypot(px - x_c, py - y_c)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        target_idx = closest_idx
        for i in range(closest_idx, len(self.path_points)):
            px, py = self.path_points[i]
            dist = math.hypot(px - x_c, py - y_c)
            if dist >= self.lookahead_dist:
                target_idx = i
                break
        if target_idx == closest_idx and target_idx == len(self.path_points) - 1:
            target_idx = len(self.path_points) - 1
        self.current_target_index = target_idx
        return self.path_points[target_idx]

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if not self._clock_initialized:
            now = self.get_clock().now()
            if now.nanoseconds == 0: return
            self.start_time = now
            self.last_advance_time = now
            self.last_log_time = now
            self._clock_initialized = True

        if not self._alignment_ready:
            self.stop_robot()
            return

        now = self.get_clock().now()
        if not self.path_points:
            if (now - self.start_time).nanoseconds / 1e9 > 3.0:
                self.replan_pub.publish(Empty())
            return
            
        if self.ruta_completada:
            return
            
        x, y, theta = self.get_current_pose()
        if x is None:
            self.v_prev *= 0.5
            self.w_prev *= 0.5
            cmd = Twist()
            cmd.linear.x = float(self.v_prev)
            cmd.angular.z = float(self.w_prev)
            self.cmd_pub.publish(cmd)
            return
            
        x_c = x + self.h * math.cos(theta)
        y_c = y + self.h * math.sin(theta)
        target = self.get_target_point(x_c, y_c)
        if not target: return
        x_d, y_d = target
        
        dist_to_final = math.hypot(self.path_points[-1][0] - x, self.path_points[-1][1] - y)

        mode_str = "PURE_ROT" if self.pure_rotation_mode else "KD"
        self.get_logger().info(
            f"[STATUS] mode={mode_str} wpt={self.current_target_index}/{len(self.path_points)} "
            f"dist_final={dist_to_final:.2f}m min_front={self.min_dist_frontal:.2f}m",
            throttle_duration_sec=5.0,
        )

        # Condición de llegada puramente por distancia al final
        if dist_to_final < 0.6:
            self.ruta_completada = True
            self.stop_robot()
            self.get_logger().info(f"Meta alcanzada (dist={dist_to_final:.3f}m). Deteniendo.")
            return

        # Watchdog
        near_final_goal = (self.current_target_index >= len(self.path_points) - 2 and dist_to_final < 0.65)
        if self.current_target_index > self.last_target_index:
            self.last_target_index = self.current_target_index
            self.last_advance_time = now
        else:
            elapsed = (now - self.last_advance_time).nanoseconds / 1e9
            timeout = 10.0 if self.pure_rotation_mode else 5.0
            if elapsed > timeout and not near_final_goal:
                self.replan_pub.publish(Empty())
                self.last_advance_time = now 

        e_x = x_d - x_c
        e_y = y_d - y_c
        
        v_dinamica = self.v_max
        # Sin boost en recta: el chassis capa a 0.25 m/s, pedir mas no sirve y desestabiliza SLAM.
        # (antes subia a 0.7 en linea recta)
            
        dist_to_target = math.hypot(e_x, e_y)
        if dist_to_target > 0:
            v_ref_x = v_dinamica * (e_x / dist_to_target)
            v_ref_y = v_dinamica * (e_y / dist_to_target)
        else:
            v_ref_x, v_ref_y = 0.0, 0.0

        k_att = 1.0
        if self.min_dist_frontal < self.umbral_frontal:
            k_att = (self.min_dist_frontal - 0.20) / (self.umbral_frontal - 0.20)
            k_att = max(0.0, min(1.0, k_att))
            
        u1 = k_att * (v_ref_x + self.k_px * e_x)
        u2 = k_att * (v_ref_y + self.k_py * e_y)

        if self.obstaculo_cerca:
            rep_mundo_x = (self.repulsion_x * math.cos(theta) - self.repulsion_y * math.sin(theta))
            rep_mundo_y = (self.repulsion_x * math.sin(theta) + self.repulsion_y * math.cos(theta))
            rep_mag = math.hypot(rep_mundo_x, rep_mundo_y)
            max_rep = 0.45
            if rep_mag > max_rep:
                rep_mundo_x = (rep_mundo_x / rep_mag) * max_rep
                rep_mundo_y = (rep_mundo_y / rep_mag) * max_rep
            u1 += rep_mundo_x
            u2 += rep_mundo_y

        # Transformacion a diferencial
        v = u1 * math.cos(theta) + u2 * math.sin(theta)
        w = (-u1 * math.sin(theta) + u2 * math.cos(theta)) / self.h
        
        # Etapa 1: Rotacion Pura
        if self.pure_rotation_mode:
            theta_d = math.atan2(y_d - y, x_d - x)
            e_theta = theta_d - theta
            while e_theta > math.pi: e_theta -= 2.0 * math.pi
            while e_theta < -math.pi: e_theta += 2.0 * math.pi

            if self._check_initial_alignment:
                self._check_initial_alignment = False
                if abs(e_theta) < 0.30:
                    self.pure_rotation_mode = False

            if self.pure_rotation_mode and abs(e_theta) > 0.15:
                v_arranque = 0.08
                v = v_arranque * max(0.0, 1.0 - abs(e_theta) / 0.5)
                
                dt = 0.05
                self.rot_integral += e_theta * dt
                self.rot_integral = max(-1.0, min(1.0, self.rot_integral))
                
                w = 1.0 * e_theta + 0.2 * self.rot_integral
                if abs(w) < 0.35: w = 0.35 if w > 0 else -0.35
            elif self.pure_rotation_mode:
                self.pure_rotation_mode = False
                self.rot_integral = 0.0
        
        v = max(min(v, v_dinamica), -v_dinamica)
        w = max(min(w, self.w_max), -self.w_max)
        
        alpha_v, alpha_w = 0.8, 0.8
        self.v_prev = (alpha_v * v) + (1.0 - alpha_v) * self.v_prev
        self.w_prev = (alpha_w * w) + (1.0 - alpha_w) * self.w_prev

        cmd = Twist()
        cmd.linear.x = float(self.v_prev)
        # linear.y SIEMPRE es 0 (diferencial puro)
        cmd.linear.y = 0.0
        cmd.angular.z = float(self.w_prev)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControlDiferencial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
