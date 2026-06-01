#!/usr/bin/env python3
# encoding: utf-8
"""
jetauto_chassis_sim.py  —  SIM twin of the REAL JetAuto base driver.

Faithful sim-to-real port of the Jetson Orin `jetauto_controller`
(`mecanum.py` + `chassis_node.py`). Instead of writing wheel speeds to the
I2C encoder-motor board (0x34), it commands the 4 simulated wheel joints
through a ros2_control JointGroupVelocityController; the modeled rollers +
ground friction then produce the holonomic motion physically.

Signal path kept identical to the real robot:
  /cmd_vel --(go_factor/turn_factor)--> mecanum inverse kinematics
           --> 4 wheel angular velocities [FL, FR, RL, RR] (rad/s)
           --> /velocity_controller/commands  (Float64MultiArray)

Odometry kept identical to the real robot (OPEN-LOOP dead-reckoning of the
command, NOT wheel encoders): /odom_raw (no TF). robot_localization EKF then
fuses /odom_raw + /imu/data (madgwick) into /odom (+ odom->base_footprint TF),
exactly like the real `chassis.launch.py`.

Real constants (Orin chassis_params.yaml / mecanum.py):
    a = 103 mm, b = 97 mm, wheel_diameter = 96.5 mm,
    go_factor = 0.90, turn_factor = 0.93.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

# Covariances copied verbatim from the real odom_publisher.py / chassis_node.py
ODOM_POSE_COV = [1e-9, 0., 0., 0., 0., 0.,
                 0., 1e-3, 1e-9, 0., 0., 0.,
                 0., 0., 1e6, 0., 0., 0.,
                 0., 0., 0., 1e6, 0., 0.,
                 0., 0., 0., 0., 1e6, 0.,
                 0., 0., 0., 0., 0., 1e3]
ODOM_TWIST_COV = [1e-9, 0., 0., 0., 0., 0.,
                  0., 1e-3, 1e-9, 0., 0., 0.,
                  0., 0., 1e6, 0., 0., 0.,
                  0., 0., 0., 1e6, 0., 0.,
                  0., 0., 0., 0., 1e6, 0.,
                  0., 0., 0., 0., 0., 0.1]


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class JetautoChassisSim(Node):
    def __init__(self):
        super().__init__('jetauto_chassis_sim')

        # --- geometry / calibration (REAL JetAuto values as defaults) ---
        self.declare_parameter('wheelbase_a', 103.0)      # mm
        self.declare_parameter('wheelbase_b', 97.0)       # mm
        self.declare_parameter('wheel_diameter', 96.5)    # mm
        self.declare_parameter('go_factor', 0.90)
        self.declare_parameter('turn_factor', 0.93)
        self.declare_parameter('linear_correction_factor', 1.0)
        self.declare_parameter('angular_correction_factor', 1.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('wheel_cmd_topic', '/velocity_controller/commands')

        gp = self.get_parameter
        a = gp('wheelbase_a').value
        b = gp('wheelbase_b').value
        self.r = (gp('wheel_diameter').value / 1000.0) / 2.0     # m  (0.04825)
        self.k = (a + b) / 1000.0                                # m  (0.200) = lx+ly (real calibrated)
        self.go_factor = gp('go_factor').value
        self.turn_factor = gp('turn_factor').value
        self.lin_corr = gp('linear_correction_factor').value
        self.ang_corr = gp('angular_correction_factor').value
        self.rate = float(gp('control_rate').value)
        self.cmd_timeout = gp('cmd_vel_timeout').value
        self.odom_frame = gp('odom_frame_id').value
        self.base_frame = gp('base_frame_id').value

        # --- state ---
        self._lock = threading.Lock()
        self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0   # already scaled by go/turn factors
        self.x = self.y = self.yaw = 0.0
        self.last_cmd_time = self.get_clock().now()

        # --- pub / sub ---
        self.pub_wheels = self.create_publisher(
            Float64MultiArray, gp('wheel_cmd_topic').value, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 10)
        self.create_subscription(Twist, gp('cmd_topic').value, self.cmd_vel_cb, 10)

        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.update)
        self.get_logger().info(
            f'jetauto_chassis_sim listo  (r={self.r:.5f} m, a+b={self.k:.3f} m, '
            f'go={self.go_factor}, turn={self.turn_factor}, {self.rate:.0f} Hz)')

    def cmd_vel_cb(self, msg: Twist):
        with self._lock:
            # identical to real chassis_node.cmd_vel_cb
            self.cmd_vx = self.go_factor * msg.linear.x
            self.cmd_vy = self.go_factor * msg.linear.y
            self.cmd_wz = self.turn_factor * msg.angular.z
            self.last_cmd_time = self.get_clock().now()

    def _wheel_velocities(self, vx, vy, wz):
        """Mecanum inverse kinematics -> wheel angular velocities [FL, FR, RL, RR] (rad/s).
        Standard X-roller config, matching the modeled-roller alpha = (xr*yr)*pi/4 and the
        real mecanum.py sign pattern (vx fwd, vy left, wz ccw)."""
        k, r = self.k, self.r
        fl = (vx - vy - k * wz) / r
        fr = (vx + vy + k * wz) / r
        rl = (vx + vy - k * wz) / r
        rr = (vx - vy + k * wz) / r
        return [fl, fr, rl, rr]

    def update(self):
        now = self.get_clock().now()
        with self._lock:
            age = (now - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                # cmd_vel watchdog (same intent as the real driver's reset)
                self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
            vx, vy, wz = self.cmd_vx, self.cmd_vy, self.cmd_wz

        # 1) drive the wheels (velocity command -> physics via rollers/friction)
        wheels = Float64MultiArray()
        wheels.data = self._wheel_velocities(vx, vy, wz)
        self.pub_wheels.publish(wheels)

        # 2) open-loop dead-reckoning odometry (identical to real chassis_node.update)
        self.x += (math.cos(self.yaw) * vx - math.sin(self.yaw) * vy) * self.dt * self.lin_corr
        self.y += (math.sin(self.yaw) * vx + math.cos(self.yaw) * vy) * self.dt * self.lin_corr
        self.yaw += wz * self.dt * self.ang_corr

        stamp = now.to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        odom.pose.covariance = ODOM_POSE_COV
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = ODOM_TWIST_COV
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = JetautoChassisSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
