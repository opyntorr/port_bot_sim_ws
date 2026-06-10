#!/usr/bin/env python3
"""
EKF manual (extendido) para el JetAuto mecanum  —  NODO ROS2 (robot real).

Corre en el Orin junto al stack real. Es un nodo AUTONOMO (rclpy, sin colcon):
    source /opt/ros/humble/setup.bash   # + el env DDS del robot (dominio 0)
    python3 ekf_manual_node.py

Fusiona (estado 4D = [x, y, theta, b_w], ver ekf_model.py):
  PREDICCION (~50 Hz, en cada /imu/data_raw):
    * vx, vy  <- /odom_raw .twist.linear   (dead-reckoning mecanum)
    * w_imu   <- /imu/data_raw .angular_velocity.z  (giro CRUDO, con bias)
    -> integra el modelo mecanum no lineal; el bias b_w se estima online.
  CORRECCION (cuando llega pose global):
    * z=[x,y,theta] <- correction_topic (PoseWithCovarianceStamped, p.ej. /amcl_pose)
    -> ancla la deriva y permite OBSERVAR el bias del giro.

Publica /odometry/ekf_manual (nav_msgs/Odometry, frame 'map'->'base_footprint').
Por defecto NO publica TF (para no chocar con robot_localization/AMCL); util para
comparar en RViz/EVO contra la odometria sola y el GT de OptiTrack.

Sin correction_topic activo, corre SOLO prediccion (deriva, igual que odom): el
valor del EKF se ve cuando hay correcciones (AMCL o scan-match) que anclen.
"""

import math
import os
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.time import Time
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ekf_model as m   # noqa: E402


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_from_yaw(th):
    return (0.0, 0.0, math.sin(th / 2.0), math.cos(th / 2.0))


class EKFManualNode(Node):
    def __init__(self):
        super().__init__('ekf_manual_node')
        gp = lambda n, v: self.declare_parameter(n, v).value

        self.frame_id = gp('frame_id', 'map')
        self.child_frame = gp('child_frame', 'base_footprint')
        self.publish_tf = gp('publish_tf', False)
        self.correction_topic = gp('correction_topic', '/amcl_pose')
        # corregir desde el TF parent->child (p.ej. slam_toolbox: map->base_footprint)
        self.use_tf_correction = gp('use_tf_correction', False)
        self.correction_parent = gp('correction_parent', 'map')
        self.tf_corr_rate = gp('tf_corr_rate', 8.0)
        # ruido de proceso (defaults de la caracterizacion; re-sintonizables)
        self.sig_vx = gp('sig_vx', 0.02)        # m/s
        self.sig_vy = gp('sig_vy', 0.02)        # m/s
        self.sig_w = gp('sig_w', 0.0)           # rad/s ; 0 => usar ARW por dt
        self.sig_b = gp('sig_b', 5.0e-4)        # rad/s/sqrt(s) (random walk del bias)
        # R por defecto si la pose no trae covarianza (sigma del MS200 escalado)
        self.def_sig_xy = gp('corr_sigma_xy', 0.05)   # m
        self.def_sig_yaw = gp('corr_sigma_yaw', 0.05)  # rad
        self.max_dt = gp('max_dt', 0.1)

        # estado
        self.x = None                 # se inicializa con la 1a correccion (o 1a IMU)
        self.P = np.diag([0.1, 0.1, 0.2, np.deg2rad(2.0) ** 2])
        self.vx = self.vy = 0.0       # ultima velocidad de cuerpo (odom)
        self.have_odom = False
        self.last_imu_t = None
        self.n_corr = 0
        self.n_imu = 0
        self.last_log = 0.0
        self.last_pred_log = 0.0

        self.create_subscription(Odometry, '/odom_raw', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Odometry, '/odometry/ekf_manual', 10)
        self.tf_bc = TransformBroadcaster(self) if self.publish_tf else None

        if self.use_tf_correction:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_timer(1.0 / self.tf_corr_rate, self.tf_corr_cb)
            corr_src = f"TF {self.correction_parent}->{self.child_frame}"
        else:
            self.create_subscription(PoseWithCovarianceStamped, self.correction_topic,
                                     self.corr_cb, 10)
            corr_src = f"topic {self.correction_topic}"

        self.get_logger().info(
            f"EKF manual listo. correccion={corr_src}, "
            f"publish_tf={self.publish_tf}. Prediccion en /imu/data_raw + /odom_raw.")

    # --- velocidades de cuerpo (dead-reckoning mecanum) ---
    def odom_cb(self, msg: Odometry):
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.have_odom = True

    # --- PREDICCION en cada muestra de IMU (~50 Hz) ---
    def imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_t is None:
            self.last_imu_t = t
            return
        dt = t - self.last_imu_t
        self.last_imu_t = t
        if dt <= 0.0 or dt > self.max_dt:
            return
        if self.x is None:
            if not self.have_odom:
                return
            self.x = np.zeros(4)        # sin correccion aun: arranca en el origen

        w_imu = msg.angular_velocity.z
        u = np.array([self.vx, self.vy, w_imu])
        sig_w = self.sig_w if self.sig_w > 0 else m.imu_arw_rate_std(dt)
        self.x, self.P = m.ekf_predict(self.x, self.P, u, dt,
                                       self.sig_vx, self.sig_vy, sig_w, self.sig_b)
        self.publish(msg.header.stamp, u)
        self.n_imu += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_pred_log > 2.0:
            self.last_pred_log = now
            self.get_logger().info(
                f"pred n_imu={self.n_imu} odom={self.have_odom} corr={self.n_corr}  "
                f"pose=({self.x[0]:+.2f},{self.x[1]:+.2f},{math.degrees(self.x[2]):+.1f}deg) "
                f"b_giro={math.degrees(self.x[3]):+.3f}deg/s")

    # --- CORRECCION (nucleo comun): z=[x,y,theta], R 3x3 ---
    def apply_correction(self, z, R):
        if self.x is None:
            self.x = np.array([z[0], z[1], z[2], 0.0])   # init con la 1a pose
            return
        self.x, self.P = m.ekf_update_pose(self.x, self.P, z, R)
        self.n_corr += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_log > 5.0:
            self.last_log = now
            self.get_logger().info(
                f"corr#{self.n_corr}  bias_giro_est={math.degrees(self.x[3]):+.3f} deg/s  "
                f"pose=({self.x[0]:+.2f},{self.x[1]:+.2f},{math.degrees(self.x[2]):+.1f}deg)")

    # --- CORRECCION via topic de pose (AMCL / scan-match) ---
    def corr_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        z = np.array([p.x, p.y, yaw_from_quat(msg.pose.pose.orientation)])
        c = msg.pose.covariance
        sxx, syy, syaw = c[0], c[7], c[35]
        R = np.diag([
            sxx if sxx > 1e-9 else self.def_sig_xy ** 2,
            syy if syy > 1e-9 else self.def_sig_xy ** 2,
            syaw if syaw > 1e-9 else self.def_sig_yaw ** 2,
        ])
        self.apply_correction(z, R)

    # --- CORRECCION via TF parent->child (slam_toolbox: map->base_footprint) ---
    def tf_corr_cb(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.correction_parent, self.child_frame, Time())
        except Exception:
            return
        t = tf.transform.translation
        z = np.array([t.x, t.y, yaw_from_quat(tf.transform.rotation)])
        R = np.diag([self.def_sig_xy ** 2, self.def_sig_xy ** 2, self.def_sig_yaw ** 2])
        self.apply_correction(z, R)

    # --- publicar /odometry/ekf_manual (+ TF opcional) ---
    def publish(self, stamp, u):
        x = self.x
        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.frame_id
        od.child_frame_id = self.child_frame
        od.pose.pose.position.x = float(x[0])
        od.pose.pose.position.y = float(x[1])
        qx, qy, qz, qw = quat_from_yaw(x[2])
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        # covarianza de pose 6x6 desde P[0:3,0:3] (x,y,yaw -> idx 0,7,35; 1,6; etc.)
        cov = [0.0] * 36
        cov[0] = float(self.P[0, 0]); cov[1] = float(self.P[0, 1]); cov[5] = float(self.P[0, 2])
        cov[6] = float(self.P[1, 0]); cov[7] = float(self.P[1, 1]); cov[11] = float(self.P[1, 2])
        cov[30] = float(self.P[2, 0]); cov[31] = float(self.P[2, 1]); cov[35] = float(self.P[2, 2])
        od.pose.covariance = cov
        # twist (cuerpo): vx,vy de odom, wz corregido por el bias estimado
        od.twist.twist.linear.x = float(u[0])
        od.twist.twist.linear.y = float(u[1])
        od.twist.twist.angular.z = float(u[2] - x[3])
        self.pub.publish(od)

        if self.tf_bc is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.frame_id
            tf.child_frame_id = self.child_frame
            tf.transform.translation.x = float(x[0])
            tf.transform.translation.y = float(x[1])
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_bc.sendTransform(tf)


def main():
    rclpy.init()
    node = EKFManualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
