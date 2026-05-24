#!/usr/bin/env python3
# encoding: utf-8
"""
Mcnamu_driver_PID.py
Lazo bajo PI + feedforward por rueda (Hernández & Almeida, J. Eng. 2024).

Cambios respecto al original:
  - PI puro sin derivada (Ec. 17 del paper: u = Kp·e + Ki·∫e dt)
  - Feedforward cinemático (supera la banda muerta ~30-50 PWM del Yahboom)
  - Asignación directa: u = u_ff + u_pi  (antes era += que acumulaba y explotaba)
  - 50 Hz (antes 20 Hz)
  - Ganancias ajustadas al rango PWM -100..100

Hardware Monster AMR (Yahboom):
  PCB: M1=FL, M2=RL, M3=FR, M4=RR
  set_motor(FL, RL, FR, RR) — orden verificado en memoria del proyecto
"""

from math import pi, copysign
import threading
import time

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray

try:
    from SunriseRobotLib import SunriseRobot
    import smbus
    bus = smbus.SMBus(0)
except ImportError:
    pass  # permite importar en la Jetson sin error


class McnamuDriverPID(Node):

    def __init__(self, name):
        super().__init__(name)
        self.car = SunriseRobot()
        self.car.set_car_type(6)
        self.car.create_receive_threading()

        self.declare_parameter('imu_link', 'imu_link')
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value

        # ── Mecánica ──────────────────────────────────────────────────────
        self.declare_parameter('Lx', 0.1)    # m — semiancho (centro → rueda en X)
        self.declare_parameter('Ly', 0.1)    # m — semilargo (centro → rueda en Y)
        self.declare_parameter('R',  0.048)  # m — radio de la rueda

        # ── PI feedback (lazo bajo, Ec. 17 paper) ─────────────────────────
        self.declare_parameter('Kp', 6.0)    # PWM / (rad/s)  ganancia proporcional
        self.declare_parameter('Ki', 2.0)    # PWM / rad      ganancia integral

        # ── Feedforward cinemático ─────────────────────────────────────────
        # Mapea velocidad angular deseada a PWM base.
        # Estima: 100 PWM / 11 rad/s ≈ 9.  Ajustar con pruebas.
        self.declare_parameter('Kff', 9.0)

        # Banda muerta hardware del Yahboom (~30-50 PWM según notas)
        self.declare_parameter('deadband', 40.0)

        Lx = self.get_parameter('Lx').get_parameter_value().double_value
        Ly = self.get_parameter('Ly').get_parameter_value().double_value
        self.R        = self.get_parameter('R').get_parameter_value().double_value
        self.Kp       = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki       = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kff      = self.get_parameter('Kff').get_parameter_value().double_value
        self.deadband = self.get_parameter('deadband').get_parameter_value().double_value
        self.mecanum_k = Lx + Ly  # = Lx + Ly para la IK mecanum

        # ── Estado control [FL, FR, RL, RR] ──────────────────────────────
        self.target_w = [0.0, 0.0, 0.0, 0.0]  # rad/s deseados
        self.integral = [0.0, 0.0, 0.0, 0.0]  # acumulador PI

        # ── Encoders ──────────────────────────────────────────────────────
        # SunriseRobotLib devuelve [M1, M2, M3, M4] = [FL, RL, FR, RR]
        self._enc_prev = list(self.car.get_motor_encoder())
        self._enc_t    = time.monotonic()
        # 1440 ticks/rev (encoder óptico incremental con reductor 9.7:1 × cuadratura)
        self._c2v      = (2.0 * pi) / 1440.0  # ticks → radianes

        # ── Pub / Sub ─────────────────────────────────────────────────────
        self.create_subscription(Twist, 'cmd_vel', self._cb_cmd, 1)
        self._pub_volt  = self.create_publisher(Float32,          'voltage',      100)
        self._pub_vel   = self.create_publisher(Twist,            'vel_raw',       50)
        self._pub_imu   = self.create_publisher(Imu,              'imu/data_raw', 100)
        self._pub_wheel = self.create_publisher(Float32MultiArray, 'wheel_vel',    50)

        # ── Test de hardware directo ──────────────────────────────────────
        # Verifica que set_motor funciona antes de arrancar el lazo
        time.sleep(0.5)
        print('[TEST] set_motor(50,50,50,50) por 1s...', flush=True)
        self.car.set_motor(50, 50, 50, 50)
        time.sleep(1.0)
        self.car.set_motor(0, 0, 0, 0)
        print('[TEST] listo', flush=True)

        # ── Timers ────────────────────────────────────────────────────────
        self._running = True
        self._pid_thread = threading.Thread(target=self._pid_thread_loop, daemon=True)
        self._pid_thread.start()
        self.create_timer(0.10, self._pub_data)   # 10 Hz — telemetría

        self.get_logger().info(
            f'McnamuDriverPID listo: Kff={self.Kff} Kp={self.Kp} Ki={self.Ki} '
            f'deadband={self.deadband} @ 50 Hz')

    # ── Cinemática inversa ─────────────────────────────────────────────────────
    def _cb_cmd(self, msg: Twist):
        """Convierte cmd_vel a velocidades angulares de rueda (rad/s)."""
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        print(f'[cmd] vx={vx:.2f} vy={vy:.2f} wz={wz:.2f}', flush=True)
        k  = self.mecanum_k
        r  = self.R
        # Compensación de drift eliminada (física corregida en URDF)
        vx_eff = vx

        # Convención estándar mecanum (+vx = adelante, +vy = izquierda, +wz = CC)
        self.target_w[0] = (vx_eff - vy - k * wz) / r   # FL
        self.target_w[1] = (vx_eff + vy + k * wz) / r   # FR
        self.target_w[2] = (vx_eff + vy - k * wz) / r   # RL
        self.target_w[3] = (vx_eff - vy + k * wz) / r   # RR

    # ── Hilo PID independiente ────────────────────────────────────────────────
    def _pid_thread_loop(self):
        while self._running:
            self._pid_loop()
            time.sleep(0.02)  # 50 Hz

    # ── Lazo PI + feedforward ─────────────────────────────────────────────────
    def _pid_loop(self):
        try:
            encs = list(self.car.get_motor_encoder())
            now  = time.monotonic()
            dt   = now - self._enc_t
            if dt < 0.005:   # ignorar si el intervalo es muy corto
                return

            # Velocidad angular medida por encoder [M1=FL, M2=RL, M3=FR, M4=RR]
            # Se reordena a [FL, FR, RL, RR] para coincidir con target_w
            denc = [encs[j] - self._enc_prev[j] for j in range(4)]
            actual_w = [
                denc[0] * self._c2v / dt,   # FL  (M1)
                denc[2] * self._c2v / dt,   # FR  (M3)
                denc[1] * self._c2v / dt,   # RL  (M2)
                denc[3] * self._c2v / dt,   # RR  (M4)
            ]
            self._enc_prev = encs
            self._enc_t    = now

            pwm = [0.0, 0.0, 0.0, 0.0]

            for i in range(4):
                w_ref = self.target_w[i]

                # Rueda en reposo → cortar integral y señal
                if abs(w_ref) < 0.05:
                    self.integral[i] = 0.0
                    pwm[i] = 0.0
                    continue

                error = w_ref - actual_w[i]

                # Integral con anti-windup (Ec. 17 del paper)
                self.integral[i] += error * dt
                self.integral[i]  = max(-40.0, min(40.0, self.integral[i]))

                # ── Control: feedforward + PI feedback ───────────────────
                # FIX HARDWARE: El feedforward suma la banda muerta como base para romper la inercia
                u_base = copysign(self.deadband, w_ref) if abs(w_ref) > 0.0 else 0.0
                u_ff   = u_base + (self.Kff * w_ref)
                
                u_pi   = self.Kp * error + self.Ki * self.integral[i]  # PI
                u_raw  = u_ff + u_pi

                pwm[i] = max(-100.0, min(100.0, u_raw))

            # set_motor(FL, RL, FR, RR) — orden PCB verificado
            fl, fr, rl, rr = int(pwm[0]), int(pwm[1]), int(pwm[2]), int(pwm[3])
            print(f'[pwm] FL={fl} FR={fr} RL={rl} RR={rr}  '
                  f'actual={[f"{w:.1f}" for w in actual_w]}', flush=True)
            self.car.set_motor(fl, rl, fr, rr)

            # Publicar velocidades reales [FL, FR, RL, RR]
            wv = Float32MultiArray()
            wv.data = actual_w
            self._pub_wheel.publish(wv)

        except Exception as e:
            self.get_logger().warn(f'pid_loop error: {e}', throttle_duration_sec=2.0)

    # ── Telemetría (10 Hz) ────────────────────────────────────────────────────
    def _pub_data(self):
        try:
            ts = Clock().now()

            volt = Float32()
            volt.data = float(self.car.get_battery_voltage())
            self._pub_volt.publish(volt)

            ax, ay, az = self.car.get_accelerometer_data()
            gx, gy, gz = self.car.get_gyroscope_data()
            imu = Imu()
            imu.header.stamp    = ts.to_msg()
            imu.header.frame_id = self.imu_link
            imu.linear_acceleration.x = -ax
            imu.linear_acceleration.y =  az
            imu.linear_acceleration.z =  ay
            imu.angular_velocity.x    = -gx
            imu.angular_velocity.y    =  gz
            imu.angular_velocity.z    =  gy
            self._pub_imu.publish(imu)

            vx, vy, wz = self.car.get_motion_data()
            twist = Twist()
            twist.linear.x  = float(vx)
            twist.linear.y  = float(vy)
            twist.angular.z = float(wz)
            self._pub_vel.publish(twist)

        except Exception as e:
            self.get_logger().warn(f'pub_data error: {e}', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    driver = McnamuDriverPID('driver_node_pid')
    try:
        rclpy.spin(driver)
    finally:
        driver._running = False
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
