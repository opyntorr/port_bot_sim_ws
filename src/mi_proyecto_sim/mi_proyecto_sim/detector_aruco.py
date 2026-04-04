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

class ArucoSlamTf(Node):
    def __init__(self):
        super().__init__('aruco_slam_tf')
        self.subscription = self.create_subscription(Image, '/uav/camera/image', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/uav/camera/aruco_3d', 10)
        self.bridge = CvBridge()
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Medidas de nuestro mapa aplanado
        self.tamano_pixel_mapa = 440
        self.resolucion_m_px = 4.4 / self.tamano_pixel_mapa

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

        if ids is not None:
            ids_detectados = [i[0] for i in ids]
            
            # Dibujar cuadritos verdes para el video
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Si vemos las 4 esquinas del mapa, podemos crear la matriz de aplanado
            if all(m in ids_detectados for m in [0, 1, 2, 3]):
                puntos_src = []
                for m in [0, 1, 2, 3]:
                    idx = ids_detectados.index(m)
                    puntos_src.append(self.obtener_centro_aruco(corners[idx]))

                pts_origen = np.array(puntos_src, dtype=np.float32)
                pts_destino = np.array([
                    [0, 0],
                    [self.tamano_pixel_mapa, 0],
                    [self.tamano_pixel_mapa, self.tamano_pixel_mapa],
                    [0, self.tamano_pixel_mapa]
                ], dtype=np.float32)

                matriz_homografia = cv2.getPerspectiveTransform(pts_origen, pts_destino)

                # Procesar carrito (4) y meta (5)
                for target_id, frame_name in [(4, 'carrito_aruco'), (5, 'meta_aruco')]:
                    if target_id in ids_detectados:
                        idx = ids_detectados.index(target_id)
                        esquinas_obj = corners[idx][0]
                        centro_x = np.mean(esquinas_obj[:, 0])
                        centro_y = np.mean(esquinas_obj[:, 1])

                        # Aplanar la coordenada
                        punto_original = np.array([[[centro_x, centro_y]]], dtype=np.float32)
                        punto_plano = cv2.perspectiveTransform(punto_original, matriz_homografia)

                        # CONVERSIÓN EXACTA A ROS 2 (Alineado con el yaml origin: [0.0, -4.4])
                        ros_x = punto_plano[0][0][0] * self.resolucion_m_px
                        ros_y = -punto_plano[0][0][1] * self.resolucion_m_px

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

                        self.tf_broadcaster.sendTransform(t)

        self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = ArucoSlamTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()