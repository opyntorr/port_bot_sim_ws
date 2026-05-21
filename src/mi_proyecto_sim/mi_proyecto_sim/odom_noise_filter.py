#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
import numpy as np
import math
import copy

class OdomNoiseFilter(Node):
    def __init__(self):
        super().__init__('odom_noise_filter')
        
        '''
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('noise_std_x', 0.02)
        self.declare_parameter('noise_std_y', 0.02)
        self.declare_parameter('noise_std_yaw', 0.015)
        self.declare_parameter('drift_x_per_sec', 0.02)
        self.declare_parameter('drift_y_per_sec', -0.015)
        self.declare_parameter('drift_yaw_per_sec', 0.015)
        '''
        
        self.declare_parameter('noise_std_x', 0.0)
        self.declare_parameter('noise_std_y', 0.0)
        self.declare_parameter('noise_std_yaw', 0.0)
        self.declare_parameter('drift_x_per_sec', 0.0)
        self.declare_parameter('drift_y_per_sec', 0.0)
        self.declare_parameter('drift_yaw_per_sec', 0.0)
        
        self.noise_std_x = self.get_parameter('noise_std_x').value
        self.noise_std_y = self.get_parameter('noise_std_y').value
        self.noise_std_yaw = self.get_parameter('noise_std_yaw').value
        self.drift_x_per_sec = self.get_parameter('drift_x_per_sec').value
        self.drift_y_per_sec = self.get_parameter('drift_y_per_sec').value
        self.drift_yaw_per_sec = self.get_parameter('drift_yaw_per_sec').value
        
        self.drift_x = 0.0
        self.drift_y = 0.0
        self.drift_yaw = 0.0
        self.last_time = None
        
        from rclpy.qos import QoSProfile
        qos = QoSProfile(depth=100)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom_raw', self.odom_cb, qos)
        self.tf_sub = self.create_subscription(TFMessage, '/tf_raw', self.tf_cb, qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("OdomNoiseFilter iniciado. Inyectando ruido en /odom y TF (TransformBroadcaster).")

    def update_drift(self, stamp):
        sec = stamp.sec + stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = sec
            return
        
        dt = sec - self.last_time
        self.last_time = sec
        
        if dt > 0 and dt < 1.0: # Evitar saltos gigantes
            # Integrar deriva constante
            self.drift_x += self.drift_x_per_sec * dt
            self.drift_y += self.drift_y_per_sec * dt
            self.drift_yaw += self.drift_yaw_per_sec * dt
            
            # Integrar ruido aleatorio (Random Walk)
            # La desviación estándar del ruido integrado debe escalar con sqrt(dt)
            self.drift_x += np.random.normal(0, self.noise_std_x * math.sqrt(dt))
            self.drift_y += np.random.normal(0, self.noise_std_y * math.sqrt(dt))
            self.drift_yaw += np.random.normal(0, self.noise_std_yaw * math.sqrt(dt))

    def apply_noise(self, x, y, yaw):
        # Aplicamos solo la deriva acumulada (la cual ya incluye el random walk).
        # Esto garantiza que la señal siga siendo continua y evitamos que
        # las derivadas (velocidades) exploten.
        nx = x + self.drift_x
        ny = y + self.drift_y
        nyaw = yaw + self.drift_yaw
        return nx, ny, nyaw

    def odom_cb(self, msg: Odometry):
        self.update_drift(msg.header.stamp)
        
        # Extraer
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        # Aplicar
        nx, ny, nyaw = self.apply_noise(x, y, yaw)
        
        # Re-empaquetar (Asegurar que el cuaternión es válido para TF2)
        out = copy.deepcopy(msg)
        out.pose.pose.position.x = nx
        out.pose.pose.position.y = ny
        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = math.sin(nyaw/2.0)
        out.pose.pose.orientation.w = math.cos(nyaw/2.0)
        
        self.odom_pub.publish(out)

    def tf_cb(self, msg: TFMessage):
        transforms_to_publish = []
        for t in msg.transforms:
            t_new = copy.deepcopy(t)
            if 'odom' in t.header.frame_id and 'base_footprint' in t.child_frame_id:
                self.update_drift(t.header.stamp)
                x = t.transform.translation.x
                y = t.transform.translation.y
                q = t.transform.rotation
                yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
                
                nx, ny, nyaw = self.apply_noise(x, y, yaw)
                
                t_new.transform.translation.x = float(nx)
                t_new.transform.translation.y = float(ny)
                t_new.transform.rotation.x = 0.0
                t_new.transform.rotation.y = 0.0
                t_new.transform.rotation.z = math.sin(nyaw/2.0)
                t_new.transform.rotation.w = math.cos(nyaw/2.0)
            transforms_to_publish.append(t_new)
            
        for transform in transforms_to_publish:
            self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomNoiseFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
