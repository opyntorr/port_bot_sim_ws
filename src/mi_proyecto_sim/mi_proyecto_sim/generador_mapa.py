#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os

class GeneradorMapaAruco(Node):
    def __init__(self):
        super().__init__('generador_mapa_aruco')
        #self.declare_parameter('use_sim_time', True)
        
        self.subscription = self.create_subscription(Image, '/uav/camera/image', self.image_callback, 10)
        self.bridge = CvBridge()
        
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        self.tamano_pixel_mapa = 440
        self.resolucion_m_px = 4.4 / self.tamano_pixel_mapa 

        self.get_logger().info("📸 Esperando la 'foto perfecta' con TODOS los marcadores (0, 1, 2, 3, 4 y 5)...")

    def obtener_centro_aruco(self, corners):
        puntos = np.array(corners).flatten()
        centro_x = int(np.mean(puntos[0::2])) 
        centro_y = int(np.mean(puntos[1::2])) 
        return [centro_x, centro_y]

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            ids_detectados = [i[0] for i in ids]
            
            # ======================================================
            # LA CLAVE: Exigir los 6 marcadores antes de hacer nada
            # ======================================================
            ids_necesarios = [0, 1, 2, 3, 4, 5]
            
            if all(marcador in ids_detectados for marcador in ids_necesarios):
                self.get_logger().info("✅ ¡Los 6 marcadores detectados simultáneamente! Procesando...")
                
                puntos_src = []
                for marcador in [0, 1, 2, 3]:
                    idx = ids_detectados.index(marcador)
                    puntos_src.append(self.obtener_centro_aruco(corners[idx]))
                
                pts_origen = np.array(puntos_src, dtype=np.float32)
                pts_destino = np.array([
                    [0, 0],                                           
                    [self.tamano_pixel_mapa, 0],                      
                    [self.tamano_pixel_mapa, self.tamano_pixel_mapa], 
                    [0, self.tamano_pixel_mapa]                       
                ], dtype=np.float32)
                
                # Aplanar y binarizar
                matriz_homografia = cv2.getPerspectiveTransform(pts_origen, pts_destino)
                mapa_plano = cv2.warpPerspective(cv_image, matriz_homografia, (self.tamano_pixel_mapa, self.tamano_pixel_mapa))
                mapa_gris = cv2.cvtColor(mapa_plano, cv2.COLOR_BGR2GRAY)
                mapa_suavizado = cv2.GaussianBlur(mapa_gris, (5, 5), 0)
                _, mapa_binario = cv2.threshold(mapa_suavizado, 110, 255, cv2.THRESH_BINARY_INV)
                
                # ======================================================
                # BORRADO SEGURO
                # ======================================================
                ids_a_borrar = [4, 5] 
                radio_borrado_px = 30 # Círculo de 60cm de diámetro
                
                for marker_corners, marker_id_arr in zip(corners, ids):
                    curr_id = marker_id_arr[0]
                    if curr_id in ids_a_borrar:
                        centro_original = self.obtener_centro_aruco(marker_corners)
                        punto_original_3d = np.array([[[centro_original[0], centro_original[1]]]], dtype=np.float32)
                        punto_plano_3d = cv2.perspectiveTransform(punto_original_3d, matriz_homografia)
                        
                        centro_x_mapa = int(punto_plano_3d[0][0][0])
                        centro_y_mapa = int(punto_plano_3d[0][0][1])
                        
                        cv2.circle(mapa_binario, (centro_x_mapa, centro_y_mapa), radio_borrado_px, 255, -1)
                        self.get_logger().info(f"✨ Mancha del ID {curr_id} eliminada con éxito.")
                
                # Guardar archivos
                directorio_actual = os.getcwd()
                cv2.imwrite(os.path.join(directorio_actual, 'mapa_laberinto.pgm'), mapa_binario)
                
                config_yaml = {
                    'image': 'mapa_laberinto.pgm',
                    'resolution': self.resolucion_m_px,
                    'origin': [0.0, -4.4, 0.0],
                    'occupied_thresh': 0.65,
                    'free_thresh': 0.196,
                    'negate': 0
                }
                with open(os.path.join(directorio_actual, 'mapa_laberinto.yaml'), 'w') as f:
                    yaml.dump(config_yaml, f, default_flow_style=False)
                
                self.get_logger().info("🎉 ¡Mapa estático LIMPIO generado!")
                raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    nodo = GeneradorMapaAruco()
    try:
        rclpy.spin(nodo)
    except SystemExit:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()