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
        
        # En ros_gz_bridge usualmente se rutea como BEST_EFFORT por el tamaño
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        
        # Publicador del escaner filtrado (usaremos BEST_EFFORT para empatar si es necesario, 
        # pero RELIABLE es por defecto en rclpy. Lo dejamos por defecto para que SLAM lo lea facilmente)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        
        self.get_logger().info("Filtro Lidar a 190º inicializado. Escuchando en /scan, publicando en /scan_filtered")

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
        
        # Angulos limite en radianes (+- 95 grados para un fov de 190)
        min_valid_angle = -95.0 * math.pi / 180.0
        max_valid_angle = 95.0 * math.pi / 180.0
        
        new_ranges = []
        new_intensities = []
        has_intensities = len(msg.intensities) > 0

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if min_valid_angle <= angle <= max_valid_angle:
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
