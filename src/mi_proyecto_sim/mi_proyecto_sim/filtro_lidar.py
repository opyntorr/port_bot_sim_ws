#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FiltroLidar(Node):
    def __init__(self):
        super().__init__('filtro_lidar')
        
        # Suscriptor al escaner nativo (con la QoS necesaria si es Gazebo)
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # Radio de auto-oclusion: en el JetAuto el lidar 2D esta rodeado de su propia
        # estructura (carcasa + postes de la camara/aruco) y el gpu_lidar la ve.
        # MEDIDO: con self_clearance=0.22 el control reportaba min_front clavado en 0.22-0.23m
        # en espacio abierto -> la estructura propia sobrevivia el umbral. Como 0.22 < umbral_frontal
        # (0.35) del control, eso mataba la atraccion (k_att~0.13) y metia repulsion hacia atras
        # -> el robot retrocedia y se bloqueaba. Subido a 0.30 (sigue < umbral_frontal=0.35, asi que
        # el control aun ve obstaculos reales antes de frenar) para descartar toda la estructura.
        # Cualquier return mas cercano que esto es el propio robot -> se descarta (inf).
        self.declare_parameter('self_clearance', 0.30)
        self.self_clearance = self.get_parameter('self_clearance').value

        # FOV util del filtro (semiangulo en grados). El recorte a +-95 (190 FOV) era para el
        # RPLIDAR A1, que solo veia ~190 (la parte trasera la tapaba el cuerpo). El MS200 ve
        # 360 reales y va montado alto sobre el soporte -> ahora 180 (= 360 completo, sin recorte
        # angular) para que la evasion del control aproveche toda la vuelta. El self_clearance
        # sigue descartando los auto-golpes del propio robot/soporte. Para volver al recorte
        # del A1: poner half_fov_deg=95.
        self.declare_parameter('half_fov_deg', 180.0)
        self.half_fov = float(self.get_parameter('half_fov_deg').value) * math.pi / 180.0

        # En ros_gz_bridge usualmente se rutea como BEST_EFFORT por el tamaño
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)

        # Publicador del escaner filtrado (usaremos BEST_EFFORT para empatar si es necesario,
        # pero RELIABLE es por defecto en rclpy. Lo dejamos por defecto para que SLAM lo lea facilmente)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)

        self.get_logger().info(
            f"Filtro Lidar a {2*math.degrees(self.half_fov):.0f}deg FOV "
            f"(self_clearance={self.self_clearance:.2f}m) inicializado. /scan -> /scan_filtered")

    def scan_callback(self, msg):
        import array
        # Construir un mensaje estrictamente nuevo para no mutar pasajes por referencia
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = float(msg.angle_min)
        filtered_msg.angle_max = float(msg.angle_max)
        filtered_msg.angle_increment = float(msg.angle_increment)
        filtered_msg.time_increment = float(msg.time_increment)
        filtered_msg.scan_time = float(msg.scan_time)
        filtered_msg.range_min = float(msg.range_min)
        filtered_msg.range_max = float(msg.range_max)
        
        # Arco util: +- half_fov (default 180 = 360 completo, sin recorte angular)
        min_valid_angle = -self.half_fov
        max_valid_angle = self.half_fov
        full_fov = self.half_fov >= math.pi - 1e-6  # 360: aceptar todos los angulos

        new_ranges = []
        new_intensities = []
        has_intensities = len(msg.intensities) > 0

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            # Fuera del arco util o dentro del radio de auto-oclusion (la propia
            # estructura del robot) -> invalido (inf).
            if (full_fov or (min_valid_angle <= angle <= max_valid_angle)) and r >= self.self_clearance:
                new_ranges.append(float(r))
                if has_intensities:
                    new_intensities.append(float(msg.intensities[i]))
            else:
                new_ranges.append(float('inf'))
                if has_intensities:
                    new_intensities.append(0.0)
                    
        # Usar array format nativo de ROS 2 para Sequence<float>
        filtered_msg.ranges = array.array('f', new_ranges)
        if has_intensities:
            filtered_msg.intensities = array.array('f', new_intensities)
            
        self.pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    nodo = FiltroLidar()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
