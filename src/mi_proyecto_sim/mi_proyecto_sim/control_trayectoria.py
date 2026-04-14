#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
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
        
        # Lector de transformaciones TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Parametros del controlador de Kelly & Diaz (Sintonizados para fidelidad)
        self.h = 0.08          # Desplazamiento del punto de interes (menor = mas apegado al centro, no bajar a 0)
        self.k_px = 2.5        # Ganancia proporcional en X (aumentada para reaccion agresiva a desviaciones)
        self.k_py = 2.5        # Ganancia proporcional en Y
        self.v_max = 0.25      # Velocidad maxima permitida (un poco menor para mas precision en esquinas)
        self.w_max = 1.2       # Velocidad angular maxima permitida

        
        self.lookahead_dist = 0.1 # Distancia para buscar el siguiente punto objetivo
        
        self.current_target_index = 0
        self.ruta_completada = False
        
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

    def get_current_pose(self):
        try:
            # Buscar transformacion desde map hasta carrito_aruco
            trans = self.tf_buffer.lookup_transform('map', 'carrito_aruco', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            q = trans.transform.rotation
            euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            theta = euler[2]
            
            # Aplicar la compensacion de 90° en reloj (-1.5708 rad) para apuntar
            # hacia el "frente verdadero" fisico del chasis.
            theta -= - math.pi / 2
            
            return x, y, theta
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
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

        # Velocidades de control en el mundo (u1, u2)
        u1 = v_ref_x + self.k_px * e_x
        u2 = v_ref_y + self.k_py * e_y
        
        # 4. Transformar a velocidades del robot diferencial (v, w)
        v = u1 * math.cos(theta) + u2 * math.sin(theta)
        w = (-u1 * math.sin(theta) + u2 * math.cos(theta)) / self.h
        
        # 5. Saturación de control para que no se vuelva loco
        v = max(min(v, self.v_max), -self.v_max)
        w = max(min(w, self.w_max), -self.w_max)
        
        # 6. Publicar comandos
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
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
