#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import tf2_ros
import cv2
import numpy as np
import math
import json
import os

class ArucoSlamTf(Node):
    def __init__(self):
        super().__init__('aruco_slam_tf')
        self.subscription = self.create_subscription(Image, '/uav/camera/image', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/uav/camera/aruco_3d', 10)
        self.bridge = CvBridge()
        
        # Broadcaster Estático: Lo que publiques aquí se queda para siempre en el árbol de TF
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        self.snapshot_tomado = False
        self.last_log_time = self.get_clock().now()
        
        # Parámetros configurables desde el launch
        self.declare_parameter('aruco_size_m', 0.12)
        self.declare_parameter('tamano_pixel_mapa', 440)
        
        self.aruco_size_m = self.get_parameter('aruco_size_m').value
        self.tamano_pixel_mapa = self.get_parameter('tamano_pixel_mapa').value
        
        self.get_logger().info(f"📸 Iniciando detector ArUco (tamaño={self.aruco_size_m}m, mapa={self.tamano_pixel_mapa}px). Esperando marcadores (0-5)...")



    def obtener_centro_aruco(self, corners):
        puntos = np.array(corners).flatten()
        return [np.mean(puntos[0::2]), np.mean(puntos[1::2])]

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        
        now = self.get_clock().now()
        if ids is not None:
            ids_detectados = [i[0] for i in ids]
            
            # Dibujar cuadritos verdes para el video de diagnóstico
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # EXIGENCIA: Ver tablero (0-3), Robot (4) y Meta (5) simultáneamente
            if not self.snapshot_tomado and all(m in ids_detectados for m in [0, 1, 2, 3, 4, 5]):
                puntos_src = []
                tamanos_raw = {}
                
                # 1. Obtener centros y tamaños raw de todos los marcadores necesarios
                for m in [0, 1, 2, 3, 4, 5]:
                    idx = ids_detectados.index(m)
                    esquinas = corners[idx][0]
                    centro = np.mean(esquinas, axis=0)
                    
                    # Calcular tamaño raw (perímetro / 4)
                    lado = (np.linalg.norm(esquinas[0]-esquinas[1]) + 
                            np.linalg.norm(esquinas[1]-esquinas[2]) + 
                            np.linalg.norm(esquinas[2]-esquinas[3]) + 
                            np.linalg.norm(esquinas[3]-esquinas[0])) / 4.0
                    tamanos_raw[m] = lado
                    
                    if m in [0, 1, 2, 3]:
                        puntos_src.append(centro)

                # Tamaño base del suelo (ArUco 0)
                S0 = tamanos_raw[0]

                pts_origen = np.array(puntos_src, dtype=np.float32)
                pts_destino = np.array([
                    [0, 0],
                    [self.tamano_pixel_mapa, 0],
                    [self.tamano_pixel_mapa, self.tamano_pixel_mapa],
                    [0, self.tamano_pixel_mapa]
                ], dtype=np.float32)

                matriz_homografia = cv2.getPerspectiveTransform(pts_origen, pts_destino)

                # --- CÁLCULO DINÁMICO DE RESOLUCIÓN ---
                # Tomamos el marcador 0 como referencia física
                idx_m0 = ids_detectados.index(0)
                esquinas_m0_orig = np.array([corners[idx_m0][0]], dtype=np.float32)
                esquinas_m0_plano = cv2.perspectiveTransform(esquinas_m0_orig, matriz_homografia)[0]
                
                # Ancho en píxeles (distancia entre esquina sup-izq y sup-der)
                w_px = np.linalg.norm(esquinas_m0_plano[0] - esquinas_m0_plano[1])
                # Alto en píxeles (distancia entre esquina sup-der e inf-der)
                h_px = np.linalg.norm(esquinas_m0_plano[1] - esquinas_m0_plano[2])
                
                # Tamaño real del marcador (parametrizado desde el launch)
                tamano_real_aruco = self.aruco_size_m
                res_x = tamano_real_aruco / w_px
                res_y = tamano_real_aruco / h_px
                
                self.get_logger().info(f"Escala calculada: res_x={res_x:.5f} m/px, res_y={res_y:.5f} m/px")

                static_transforms = []

                # Procesar carrito (4) y meta (5)
                img_h, img_w = cv_image.shape[:2]
                cam_cx, cam_cy = img_w / 2.0, img_h / 2.0
                
                for target_id, frame_name in [(4, 'carrito_aruco'), (5, 'meta_aruco')]:
                    idx = ids_detectados.index(target_id)
                    esquinas_obj = corners[idx][0]
                    centro_x = np.mean(esquinas_obj[:, 0])
                    centro_y = np.mean(esquinas_obj[:, 1])

                    # --- CORRECCIÓN DE PARALAJE AUTOMÁTICA ---
                    # Ratio mágico: si el marcador se ve más grande, está más alto.
                    # Empujamos su coordenada hacia el centro de la lente por ese ratio.
                    Sm = tamanos_raw[target_id]
                    ratio = S0 / Sm
                    
                    cx_true = cam_cx + (centro_x - cam_cx) * ratio
                    cy_true = cam_cy + (centro_y - cam_cy) * ratio

                    # Aplanar la coordenada YA corregida
                    punto_original = np.array([[[cx_true, cy_true]]], dtype=np.float32)
                    punto_plano = cv2.perspectiveTransform(punto_original, matriz_homografia)

                    # CONVERSIÓN CON RESOLUCIÓN DINÁMICA
                    ros_x = float(punto_plano[0][0][0] * res_x)
                    ros_y = float(-punto_plano[0][0][1] * res_y)

                    # Calcular Yaw
                    vector_frente_orig = np.array([[esquinas_obj[0], esquinas_obj[1]]], dtype=np.float32)
                    vector_frente_plano = cv2.perspectiveTransform(vector_frente_orig, matriz_homografia)
                    dx = vector_frente_plano[0][1][0] - vector_frente_plano[0][0][0]
                    dy = vector_frente_plano[0][1][1] - vector_frente_plano[0][0][1]
                    
                    yaw = math.atan2(-dy, dx)

                    # Publicar TF
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'map'
                    t.child_frame_id = frame_name
                    t.transform.translation.x = float(ros_x)
                    t.transform.translation.y = float(ros_y)
                    t.transform.translation.z = 0.0

                    q = self.quaternion_from_euler(0, 0, yaw)
                    t.transform.rotation.x = float(q[0])
                    t.transform.rotation.y = float(q[1])
                    t.transform.rotation.z = float(q[2])
                    t.transform.rotation.w = float(q[3])

                    static_transforms.append(t)

                # Publicar TODAS las TFs estáticas de golpe
                self.tf_static_broadcaster.sendTransform(static_transforms)
                


                self.snapshot_tomado = True
                self.get_logger().info("✅ ¡Snapshot Exitoso! TFs de Meta y Posición Inicial fijadas.")
                self.get_logger().info("🛑 Snapshot completado. Se seguirá publicando video, pero sin recalcular TFs.")

            elif not self.snapshot_tomado:
                # Log de diagnóstico: Qué falta para el snapshot
                missing = [m for m in [0, 1, 2, 3, 4, 5] if m not in ids_detectados]
                if (now - self.last_log_time).nanoseconds / 1e9 > 2.0:
                    self.get_logger().info(f"Snapshot pendiente. Faltan marcadores: {missing}. Vistos: {ids_detectados}")
                    self.last_log_time = now
        else:
            # Log de diagnóstico: No se ve nada
            if not self.snapshot_tomado and (now - self.last_log_time).nanoseconds / 1e9 > 2.0:
                self.get_logger().info("Buscando marcadores... No se detecta ninguno en el frame.")
                self.last_log_time = now

        self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ArucoSlamTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()