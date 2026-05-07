#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.srv import LoadMap

class GeneradorMapaAruco(Node):
    def __init__(self):
        super().__init__('generador_mapa_aruco')
        #self.declare_parameter('use_sim_time', True)
        
        self.subscription = self.create_subscription(Image, '/uav/camera/image', self.image_callback, 10)
        self.bridge = CvBridge()
        
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.parameters = cv2.aruco.DetectorParameters_create()
            self.detector = None
        except AttributeError:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # GROUND TRUTH DEL LABERINTO:
        # Medida física real entre los marcadores 0 y 1 (Ancho) y 0 y 3 (Alto).
        # Para la vida real, saca tu cinta métrica, mide el rectángulo que forman, 
        # y pon esos valores exactos aquí.
        self.ancho_real_laberinto_m = 2.80
        self.alto_real_laberinto_m = 3.40
        
        # Parámetros Intrínsecos de la cámara del dron (según dronCamara.sdf)
        # width=1280, height=720, hfov=1.4416
        # f = (1280/2) / tan(1.4416/2) ≈ 728.26
        self.K = np.array([
            [728.26, 0.0, 640.0],
            [0.0, 728.26, 360.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        
        # Coeficientes de distorsión [k1, k2, p1, p2, k3]
        self.D = np.array([-0.05, 0.01, 0.0, 0.0, 0.0], dtype=np.float32)

        self.get_logger().info("Esperando la 'foto perfecta' con TODOS los marcadores (0, 1, 2, 3, 4 y 5)...")

    def obtener_centro_y_escala(self, corners):
        puntos = np.array(corners).reshape((4, 2))
        centro_x = int(np.mean(puntos[:, 0])) 
        centro_y = int(np.mean(puntos[:, 1])) 
        
        # Calcular el tamaño del marcador en píxeles (promedio de los lados)
        lado1 = np.linalg.norm(puntos[0] - puntos[1])
        lado2 = np.linalg.norm(puntos[1] - puntos[2])
        lado3 = np.linalg.norm(puntos[2] - puntos[3])
        lado4 = np.linalg.norm(puntos[3] - puntos[0])
        px_per_02m = (lado1 + lado2 + lado3 + lado4) / 4.0
        
        return [centro_x, centro_y], px_per_02m

    def image_callback(self, msg):
        raw_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # NUEVO: Corregir distorsión de la lente antes de detectar nada
        cv_image = cv2.undistort(raw_image, self.K, self.D)
        
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            ids_detectados = [i[0] for i in ids]
            
            # ======================================================
            # LA CLAVE: Exigir los 6 marcadores antes de hacer nada
            # ======================================================
            ids_necesarios = [0, 1, 2, 3, 4, 5]
            
            if all(marcador in ids_detectados for marcador in ids_necesarios):
                self.get_logger().info("¡Los 6 marcadores detectados simultáneamente! Procesando...")
                
                puntos_src = []
                escalas = []
                for marcador in [0, 1, 2, 3]:
                    idx = ids_detectados.index(marcador)
                    centro, esc = self.obtener_centro_y_escala(corners[idx])
                    puntos_src.append(centro)
                    escalas.append(esc)
                
                # GROUND TRUTH: Usamos las dimensiones físicas reales para forzar 
                # a la homografía a corregir la perspectiva, eliminando el "escorzo".
                ancho_m = self.ancho_real_laberinto_m
                alto_m = self.alto_real_laberinto_m
                
                # Definir resolución deseada (1cm/px es estándar y compatible con tu RRT)
                self.resolucion_m_px = 0.01 
                
                # Definir margen exterior de 1.0 metros
                margen_m = 1.0
                margen_px = int(margen_m / self.resolucion_m_px)
                
                width_px = int((ancho_m + 2 * margen_m) / self.resolucion_m_px)
                height_px = int((alto_m + 2 * margen_m) / self.resolucion_m_px)
                
                self.get_logger().info(f"Área ArUco: {ancho_m:.2f}m x {alto_m:.2f}m. Mapa total (con margen): {ancho_m+2*margen_m:.2f}m x {alto_m+2*margen_m:.2f}m ({width_px}x{height_px} px)")

                pts_origen = np.array(puntos_src, dtype=np.float32)
                
                # Mapear los ArUcos desplazados por el margen
                pts_destino = np.array([
                    [margen_px, margen_px],                                           
                    [margen_px + int(ancho_m / self.resolucion_m_px), margen_px],                      
                    [margen_px + int(ancho_m / self.resolucion_m_px), margen_px + int(alto_m / self.resolucion_m_px)], 
                    [margen_px, margen_px + int(alto_m / self.resolucion_m_px)]                       
                ], dtype=np.float32)
                
                # Aplanar y binarizar
                matriz_homografia = cv2.getPerspectiveTransform(pts_origen, pts_destino)
                mapa_plano = cv2.warpPerspective(cv_image, matriz_homografia, (width_px, height_px))
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
                        centro_original, _ = self.obtener_centro_y_escala(marker_corners)
                        punto_original_3d = np.array([[[centro_original[0], centro_original[1]]]], dtype=np.float32)
                        punto_plano_3d = cv2.perspectiveTransform(punto_original_3d, matriz_homografia)
                        
                        centro_x_mapa = int(punto_plano_3d[0][0][0])
                        centro_y_mapa = int(punto_plano_3d[0][0][1])
                        
                        cv2.circle(mapa_binario, (centro_x_mapa, centro_y_mapa), radio_borrado_px, 255, -1)
                        self.get_logger().info(f"Mancha del ID {curr_id} eliminada con éxito.")
                
                # Guardar archivos en la carpeta de mapas del paquete
                try:
                    pkg_sim = get_package_share_directory('mi_proyecto_sim')
                    # install/mi_proyecto_sim/share/mi_proyecto_sim -> 4 niveles arriba -> ws_root
                    ws_root = os.path.abspath(os.path.join(pkg_sim, '..', '..', '..', '..'))
                    dir_mapas = os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'maps')
                    
                    if not os.path.exists(dir_mapas):
                        os.makedirs(dir_mapas, exist_ok=True)
                except Exception:
                    # Fallback si falla el descubrimiento del paquete
                    dir_mapas = os.getcwd()

                path_pgm = os.path.join(dir_mapas, 'mapa_laberinto.pgm')
                path_yaml = os.path.join(dir_mapas, 'mapa_laberinto.yaml')

                cv2.imwrite(path_pgm, mapa_binario)
                
                config_yaml = {
                    'image': 'mapa_laberinto.pgm',
                    'resolution': self.resolucion_m_px,
                    'origin': [-float(margen_m), -float(alto_m + margen_m), 0.0],
                    'occupied_thresh': 0.65,
                    'free_thresh': 0.196,
                    'negate': 0
                }
                with open(path_yaml, 'w') as f:
                    yaml.dump(config_yaml, f, default_flow_style=False)
                
                # NUEVO: Pedirle al Map Server que recargue el mapa en caliente
                self.get_logger().info("Solicitando recarga del mapa al Map Server...")
                cli = self.create_client(LoadMap, '/map_server/load_map')
                while not cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Servicio /map_server/load_map no disponible, esperando...')
                
                req = LoadMap.Request()
                req.map_url = path_yaml
                cli.call_async(req)
                
                self.get_logger().info(f"¡Mapa estático LIMPIO generado y recarga solicitada!")
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