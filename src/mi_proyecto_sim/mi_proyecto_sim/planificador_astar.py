#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import cv2
import numpy as np
import heapq

class NodoAStar:
    def __init__(self, pos, padre=None):
        self.pos = pos
        self.padre = padre
        self.g = 0
        self.h = 0
        self.f = 0
    def __lt__(self, otro): return self.f < otro.f

class PlanificadorRuta(Node):
    def __init__(self):
        super().__init__('planificador_astar')
        #self.declare_parameter('use_sim_time', True)
        
        # Publicador de la ruta para RViz
        self.publisher_path = self.create_publisher(Path, '/plan_global', 10)
        
        # Escuchador de TFs para saber dónde están el robot y la meta
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Cargar mapa y prepararlo (Inflar paredes)
        self.mapa = cv2.imread('/ros2_ws/mapa_laberinto.pgm', cv2.IMREAD_GRAYSCALE)
        kernel = np.ones((18, 18), np.uint8) # Inflado de 18cm para seguridad
        self.mapa_inflado = cv2.erode(self.mapa, kernel, iterations=1)
        
        self.resolucion = 0.01 # 1px = 1cm
        self.timer = self.create_timer(1.0, self.calcular_y_publicar_ruta)
        self.get_logger().info("Planificador A* iniciado y esperando TFs...")

    def metro_a_pixel(self, x, y):
        px = int(x / self.resolucion)
        py = int(-y / self.resolucion) # Invertimos Y por el sistema del yaml
        return (px, py)

    def pixel_a_metro(self, px, py):
        x = float(px * self.resolucion)
        y = float(-py * self.resolucion)
        return (x, y)

    def calcular_y_publicar_ruta(self):
        try:
            # 1. Obtener posiciones actuales desde TF
            now = rclpy.time.Time()
            try:
                # Intentar obtener la TF dinámica del robot
                tf_car = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
            except Exception:
                # Fallback a la snapshot inicial del drone
                tf_car = self.tf_buffer.lookup_transform('map', 'carrito_aruco', now)
            
            tf_goal = self.tf_buffer.lookup_transform('map', 'meta_aruco', now)
            
            inicio = self.metro_a_pixel(tf_car.transform.translation.x, tf_car.transform.translation.y)
            meta = self.metro_a_pixel(tf_goal.transform.translation.x, tf_goal.transform.translation.y)
            
            # 2. Ejecutar A*
            ruta_px = self.algoritmo_astar(inicio, meta)
            
            if ruta_px:
                # 3. Convertir a mensaje de ROS 2 Path
                msg_path = Path()
                msg_path.header.frame_id = 'map'
                msg_path.header.stamp = self.get_clock().now().to_msg()
                
                for px, py in ruta_px:
                    pose = PoseStamped()
                    mx, my = self.pixel_a_metro(px, py)
                    pose.pose.position.x = mx
                    pose.pose.position.y = my
                    msg_path.poses.append(pose)
                
                self.publisher_path.publish(msg_path)
                
        except Exception as e:
            self.get_logger().warn(f"Esperando TFs... {e}")

    def algoritmo_astar(self, inicio, meta):
        lista_abierta = []
        heapq.heappush(lista_abierta, NodoAStar(inicio))
        cerrada = set()
        alto, ancho = self.mapa_inflado.shape
        
        while lista_abierta:
            actual = heapq.heappop(lista_abierta)
            if abs(actual.pos[0]-meta[0]) < 4 and abs(actual.pos[1]-meta[1]) < 4:
                camino = []
                while actual:
                    camino.append(actual.pos)
                    actual = actual.padre
                return camino[::-1]
            
            cerrada.add(actual.pos)
            for dx, dy in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
                vecino_pos = (actual.pos[0]+dx, actual.pos[1]+dy)
                if 0<=vecino_pos[0]<ancho and 0<=vecino_pos[1]<alto:
                    if self.mapa_inflado[vecino_pos[1], vecino_pos[0]] > 200 and vecino_pos not in cerrada:
                        vecino = NodoAStar(vecino_pos, actual)
                        vecino.g = actual.g + (1.4 if dx!=0 and dy!=0 else 1.0)
                        vecino.h = ((vecino_pos[0]-meta[0])**2 + (vecino_pos[1]-meta[1])**2)**0.5
                        vecino.f = vecino.g + vecino.h
                        heapq.heappush(lista_abierta, vecino)
        return None

def main():
    rclpy.init()
    node = PlanificadorRuta()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__': main()