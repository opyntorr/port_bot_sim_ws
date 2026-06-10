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
  /cmd_vel --(go_factor/turn_factor, clamp velocidad, limite de aceleracion)--> mecanum IK
           --> 4 wheel angular velocities [FL, FR, RL, RR] (rad/s)
           --> /velocity_controller/commands  (Float64MultiArray)

Limites sim-to-real (defaults, aplican a cualquier launch/run): v<=0.20 m/s y w<=0.5 rad/s
(caps reales del control_diferencial); aceleracion 1.3 m/s^2 lineal y 3.4/4.5 rad/s^2
angular (accel/decel del EKF real, use_control=true).

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

        # --- limites de velocidad/aceleracion (envolvente del JetAuto REAL; sim-to-real) ---
        # Velocidad: caps reales del control_diferencial (0.20 m/s, 0.5 rad/s).
        # Aceleracion: del EKF real (use_control=true) acceleration_limits/deceleration_limits
        #   = 1.3 m/s^2 lineal (accel=decel) y 3.4/4.5 rad/s^2 angular (accel/decel).
        # Defaults aqui => aplican a CUALQUIER launch/run del sim sin pasar params.
        self.declare_parameter('max_linear_vel', 0.20)     # m/s   (magnitud de vx,vy)
        self.declare_parameter('max_angular_vel', 0.5)     # rad/s (|wz|)
        self.declare_parameter('max_linear_accel', 1.3)    # m/s^2
        self.declare_parameter('max_linear_decel', 1.3)    # m/s^2
        self.declare_parameter('max_angular_accel', 3.4)   # rad/s^2
        self.declare_parameter('max_angular_decel', 4.5)   # rad/s^2

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
        self.max_lin_vel = gp('max_linear_vel').value
        self.max_ang_vel = gp('max_angular_vel').value
        self.max_lin_acc = gp('max_linear_accel').value
        self.max_lin_dec = gp('max_linear_decel').value
        self.max_ang_acc = gp('max_angular_accel').value
        self.max_ang_dec = gp('max_angular_decel').value

        # --- state ---
        self._lock = threading.Lock()
        # cmd_* = TARGET (escalado por go/turn + recortado al tope de velocidad real);
        # v*    = comando REAL tras el limitador de aceleracion (mueve ruedas y alimenta odom).
        self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
        self.vx = self.vy = self.wz = 0.0
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
        self.get_logger().info(
            f'  limites sim-to-real: v<=({self.max_lin_vel} m/s, {self.max_ang_vel} rad/s), '
            f'accel lin {self.max_lin_acc}/{self.max_lin_dec}, ang {self.max_ang_acc}/{self.max_ang_dec} m·rad/s^2')

    def cmd_vel_cb(self, msg: Twist):
        # escala como el chasis real (go/turn factor) y recorta al tope de velocidad real.
        vx = self.go_factor * msg.linear.x
        vy = self.go_factor * msg.linear.y
        wz = self.turn_factor * msg.angular.z
        speed = math.hypot(vx, vy)
        if speed > self.max_lin_vel and speed > 1e-9:
            s = self.max_lin_vel / speed
            vx *= s
            vy *= s
        wz = max(-self.max_ang_vel, min(self.max_ang_vel, wz))
        with self._lock:
            self.cmd_vx, self.cmd_vy, self.cmd_wz = vx, vy, wz   # TARGET (lo alcanza el slew)
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

    @staticmethod
    def _slew(current, target, accel, decel, dt):
        """Limita la tasa de cambio current->target: usa accel si aumenta la magnitud
        (misma direccion) y decel si la reduce o invierte. = limitador de aceleracion real."""
        if abs(target) > abs(current) and target * current >= 0.0:
            rate = accel
        else:
            rate = decel
        max_step = rate * dt
        diff = target - current
        if diff > max_step:
            return current + max_step
        if diff < -max_step:
            return current - max_step
        return target

    def update(self):
        now = self.get_clock().now()
        with self._lock:
            age = (now - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                # watchdog: hard-stop como el chasis real (mecanum.reset())
                self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
                self.vx = self.vy = self.wz = 0.0
            tvx, tvy, twz = self.cmd_vx, self.cmd_vy, self.cmd_wz

        # limitador de aceleracion (slew-rate) hacia el target -> comando real vx,vy,wz
        self.vx = self._slew(self.vx, tvx, self.max_lin_acc, self.max_lin_dec, self.dt)
        self.vy = self._slew(self.vy, tvy, self.max_lin_acc, self.max_lin_dec, self.dt)
        self.wz = self._slew(self.wz, twz, self.max_ang_acc, self.max_ang_dec, self.dt)
        vx, vy, wz = self.vx, self.vy, self.wz

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
