#!/usr/bin/env python3
# encoding: utf-8
"""
Mcnamu_driver_PID_sim.py
Versión adaptada para simulación (Camino A).
Se comunica con Gazebo mediante tópicos de ROS 2 en lugar de SunriseRobotLib.
"""

from math import pi, copysign
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float32MultiArray

class McnamuDriverPIDSim(Node):
    def __init__(self, name):
        super().__init__(name)

        # Nombres de las juntas en tu URDF/SDF (ajústalos según tu modelo)
        self.declare_parameter('joint_fl', 'wheel_fl_joint')
        self.declare_parameter('joint_fr', 'wheel_fr_joint')
        self.declare_parameter('joint_rl', 'wheel_rl_joint')
        self.declare_parameter('joint_rr', 'wheel_rr_joint')

        self.joint_names = [
            self.get_parameter('joint_fl').value,
            self.get_parameter('joint_fr').value,
            self.get_parameter('joint_rl').value,
            self.get_parameter('joint_rr').value
        ]

        # ── Mecánica y Parámetros PID ──────────────────────────────────────
        self.declare_parameter('Lx', 0.1)
        self.declare_parameter('Ly', 0.1)
        self.declare_parameter('R',  0.048)
        
        # Ahora usaremos las MISMAS ganancias que el robot real (PWM -100 a 100).
        # El mapeo de PWM a Torque se hará internamente.
        self.declare_parameter('Kp', 6.0)
        self.declare_parameter('Ki', 2.0)
        self.declare_parameter('Kff', 9.0)
        self.declare_parameter('deadband', 32.0)

        self.Lx = self.get_parameter('Lx').value
        self.Ly = self.get_parameter('Ly').value
        self.R = self.get_parameter('R').value
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kff = self.get_parameter('Kff').value
        self.deadband = self.get_parameter('deadband').value
        self.mecanum_k = self.Lx + self.Ly

        # Estado de control
        self.target_w = [0.0, 0.0, 0.0, 0.0]  # [FL, FR, RL, RR] rad/s
        self.actual_w = [0.0, 0.0, 0.0, 0.0]  # [FL, FR, RL, RR] rad/s leídos de Gazebo
        self.integral = [0.0, 0.0, 0.0, 0.0]

        # ── Pub / Sub ─────────────────────────────────────────────────────
        # Suscripción a comandos de velocidad general
        self.create_subscription(Twist, 'cmd_vel', self._cb_cmd, 1)
        
        # Suscripción a los encoders simulados de Gazebo
        self.create_subscription(JointState, 'joint_states', self._cb_joint_states, 10)
        
        # Publicador de esfuerzo (torque) hacia los controladores de Gazebo
        # Asume un JointGroupEffortController montado en /effort_controller/commands
        self.pub_effort = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        
        # Telemetría para graficar
        self.pub_wheel_vel = self.create_publisher(Float32MultiArray, 'wheel_vel', 50)

        # Lazo de control a 50 Hz usando un Timer de ROS 2 (más robusto que threading)
        self.timer_period = 0.02
        self.create_timer(self.timer_period, self._pid_loop)

        self.get_logger().info('McnamuDriverPID (Simulación) iniciado a 50Hz')

    def _cb_cmd(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        k = self.mecanum_k
        r = self.R
        
        # Compensación de drift eliminada (física corregida en URDF)
        vx_eff = vx

        
        # Cinemática inversa Mecanum [FL, FR, RL, RR]
        self.target_w[0] = (vx_eff - vy - k * wz) / r
        self.target_w[1] = (vx_eff + vy + k * wz) / r
        self.target_w[2] = (vx_eff + vy - k * wz) / r
        self.target_w[3] = (vx_eff - vy + k * wz) / r

    def _cb_joint_states(self, msg: JointState):
        """Lee la velocidad real de las ruedas desde Gazebo."""
        for i, j_name in enumerate(self.joint_names):
            if j_name in msg.name:
                idx = msg.name.index(j_name)
                # Gazebo publica la velocidad directamente en rad/s
                self.actual_w[i] = msg.velocity[idx]

    def _pid_loop(self):
        dt = self.timer_period
        efforts = [0.0, 0.0, 0.0, 0.0]

        for i in range(4):
            w_ref = self.target_w[i]
            w_act = self.actual_w[i]

            if abs(w_ref) < 0.05:
                self.integral[i] = 0.0
                efforts[i] = 0.0
                continue

            error = w_ref - w_act
            self.integral[i] += error * dt
            self.integral[i] = max(-10.0, min(10.0, self.integral[i])) # Límite integral

            # Feedforward + PI
            u_ff = self.Kff * w_ref
            u_pi = self.Kp * error + self.Ki * self.integral[i]
            u_raw = u_ff + u_pi

            # Banda muerta
            if 0.0 < abs(u_raw) < self.deadband:
                u_raw = copysign(self.deadband, u_raw)

            # En el robot real, el PWM va de -100 a 100.
            pwm_val = max(-100.0, min(100.0, u_raw))
            
            # SIM-TO-REAL: Mapeo de PWM a Torque (Esfuerzo).
            # Asumimos que 100 PWM = ~1.5 Nm de torque de pérdida (stall torque)
            max_torque = 1.5
            efforts[i] = (pwm_val / 100.0) * max_torque

        # Publicar comandos al controlador de esfuerzo de Gazebo
        msg_effort = Float64MultiArray()
        msg_effort.data = efforts
        self.pub_effort.publish(msg_effort)

        # Publicar velocidades reales para graficar/debug
        wv = Float32MultiArray()
        wv.data = [float(w) for w in self.actual_w]
        self.pub_wheel_vel.publish(wv)

def main():
    rclpy.init()
    node = McnamuDriverPIDSim('driver_node_pid_sim')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
