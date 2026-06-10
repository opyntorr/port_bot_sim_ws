#!/usr/bin/env python3
"""
TAREA 1 en Isaac Sim (SIM-TO-REAL) — Filtro de Kalman con ruido REAL de sensores.

Mismo filtro/control que la version de Gazebo (Kalman-Bucy continuo-discreto +
control de seguimiento Modelo 1 de Kelly); solo cambia DE DONDE salen las señales,
porque en Isaac la base es CINEMATICA (teleport desde /cmd_vel) y las ruedas son
pasivas -> /joint_states NO da velocidad util. La fuente propioceptiva se toma del
twist del simulador, que esta en marco CUERPO (verificado: lin.x = avance, ang.z = giro).

  PREDICCION (propioceptiva, 50 Hz):
    * "Encoders" = /odom.twist.linear.x  + ruido caracterizado de encoders
    * IMU yaw    = /odom.twist.angular.z + bias 0.199 deg/s + ARW
    -> dead-reckoning: el bias del giro hace DERIVAR el rumbo (como en el robot real).
  CORRECCION (exteroceptiva, 15 Hz):
    * LiDAR -> pose absoluta tipo AMCL = ground truth (/odom.pose) + ruido del MS200
    -> ANCLA la deriva.
  GROUND TRUTH (referencia): /odom.pose. NO entra al control.

Correr (host, NO docker):
    source /opt/ros/humble/setup.bash
    source isaac/isaac_env.sh            # dominio 30 + cyclonedds loopback
    python3 isaac/kf_control_isaac.py    # con scene_agv.py ya corriendo
"""

import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rover_model as rm
import sensor_models as sm

HERE = os.path.dirname(os.path.abspath(__file__))


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class KFControlIsaac(Node):
    def __init__(self):
        # use_sim_time: el control corre en tiempo de simulacion (Isaac /clock),
        # asi la trayectoria deseada y la integracion del robot van al mismo reloj.
        super().__init__('kf_control_isaac',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        gp = lambda n, v: self.declare_parameter(n, v).value

        self.h = gp('h', 0.15)
        kp = gp('kp', 0.8)
        self.Kp = np.diag([kp, kp])
        self.R_traj = gp('traj_radius', 0.6)
        self.omega_t = gp('traj_omega', 2 * np.pi / 70.0)
        self.duration = gp('duration_s', 90.0)
        self.rate = gp('control_rate', sm.IMU_RATE_HZ)
        self.out_dir = gp('output_dir', os.path.join(HERE, 'figs'))
        self.dt = 1.0 / self.rate
        self.lidar_every = max(1, int(round(self.rate / sm.LIDAR_RATE_HZ)))

        # covarianzas DERIVADAS de la caracterizacion
        self.sig_v = sm.enc_linear_std(self.dt)
        self.sig_w = sm.imu_arw_rate_std(self.dt)
        self.R_lidar = np.diag([sm.LIDAR_POSE_SIGMA_XY**2,
                                sm.LIDAR_POSE_SIGMA_XY**2,
                                sm.LIDAR_POSE_SIGMA_YAW**2])

        # estados
        self.xi_hat = None
        self.xi_odo = None
        self.P = np.diag([0.02, 0.02, 0.02])
        self.gt = None             # ground truth pose (de /odom.pose)
        self.gt_v = 0.0            # ground truth avance (de /odom.twist.linear.x, marco cuerpo)
        self.gt_w = 0.0            # ground truth yaw rate (de /odom.twist.angular.z)
        self.v_cmd = self.w_cmd = 0.0
        self.center = None
        self.phi = 0.0
        self.t0 = None
        self.k = 0
        self.rng = np.random.default_rng(0)
        self.log = []
        self.saved = False

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.dt, self.step)
        self.get_logger().info('kf_control_isaac (sim-to-real) listo...')

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.gt = np.array([p.x, p.y, yaw_from_quat(msg.pose.pose.orientation)])
        self.gt_v = msg.twist.twist.linear.x      # marco cuerpo (verificado)
        self.gt_w = msg.twist.twist.angular.z

    def desired(self, t):
        # circulo alineado al rumbo inicial: la velocidad deseada en t=0 apunta a theta0,
        # asi el control arranca sin transitorio (independiente del yaw de spawn).
        R, w, phi = self.R_traj, self.omega_t, self.phi
        qd = self.center + np.array([R * np.cos(w * t + phi), R * np.sin(w * t + phi)])
        qd_dot = np.array([-R * w * np.sin(w * t + phi), R * w * np.cos(w * t + phi)])
        return qd, qd_dot

    def step(self):
        if self.gt is None:
            return
        if self.xi_hat is None:
            self.xi_hat = self.gt.copy()
            self.xi_odo = self.gt.copy()
            th0 = self.gt[2]
            self.phi = th0 - np.pi / 2.0                       # qd_dot(0) ∝ rumbo inicial
            qp0 = rm.point_p(self.gt, self.h)
            self.center = qp0 - self.R_traj * np.array([np.cos(self.phi), np.sin(self.phi)])
            self.t0 = self.get_clock().now().nanoseconds * 1e-9
            return

        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        dt = self.dt

        # --- SENSORES con ruido caracterizado (fuente: twist del simulador) ---
        v_enc = sm.corrupt_encoder_forward(self.gt_v, self.rng, dt)   # encoders
        w_imu = sm.corrupt_imu_yaw_rate(self.gt_w, self.rng, dt)      # IMU (bias+ARW)

        # --- PREDICCION del KF (continua, Euler) con la odometria ruidosa ---
        F = rm.jacobian_unicycle(self.xi_hat, v_enc, w_imu)
        th = self.xi_hat[2]
        G = np.array([[np.cos(th), 0.0], [np.sin(th), 0.0], [0.0, 1.0]])
        Q = G @ np.diag([self.sig_v**2, self.sig_w**2]) @ G.T / dt
        Q[2, 2] += sm.IMU_GYRO_Z_BIAS**2
        Q[0, 0] += 0.02**2
        Q[1, 1] += 0.02**2
        self.xi_hat = self.xi_hat + dt * rm.f_unicycle(self.xi_hat, v_enc, w_imu)
        self.P = self.P + dt * (F @ self.P + self.P @ F.T + Q)
        self.P = 0.5 * (self.P + self.P.T)
        self.xi_odo = self.xi_odo + dt * rm.f_unicycle(self.xi_odo, v_enc, w_imu)

        # --- CORRECCION del KF (discreta) con la pose del LiDAR @ 15 Hz ---
        if self.k % self.lidar_every == 0:
            y = sm.corrupt_lidar_pose(self.gt.copy(), self.rng)
            S = self.P + self.R_lidar
            Kk = self.P @ np.linalg.inv(S)
            innov = y - self.xi_hat
            innov[2] = math.atan2(math.sin(innov[2]), math.cos(innov[2]))
            self.xi_hat = self.xi_hat + Kk @ innov
            self.P = (np.eye(3) - Kk) @ self.P
            self.P = 0.5 * (self.P + self.P.T)
        self.k += 1

        # --- CONTROL con estado estimado ---
        q_p = rm.point_p(self.xi_hat, self.h)
        qd, qd_dot = self.desired(t)
        v, w = rm.controller_twist(q_p, self.xi_hat[2], qd, qd_dot, self.Kp, self.h)
        self.v_cmd = float(np.clip(v, -sm.V_MAX, sm.V_MAX))
        self.w_cmd = float(np.clip(w, -sm.W_MAX, sm.W_MAX))
        msg = Twist()
        msg.linear.x = self.v_cmd
        msg.angular.z = self.w_cmd
        self.pub.publish(msg)

        gp_ = rm.point_p(self.gt, self.h)
        op_ = rm.point_p(self.xi_odo, self.h)
        hp_ = q_p
        self.log.append([t, *gp_, self.gt[2], *op_, self.xi_odo[2],
                         *hp_, self.xi_hat[2], *qd, *np.diag(self.P)])

        if t >= self.duration and not self.saved:
            self.pub.publish(Twist())
            self.save_plots()
            self.saved = True
            self.get_logger().info('Validacion sim-to-real terminada; figuras guardadas.')

    def save_plots(self):
        if not self.log:
            return
        os.makedirs(self.out_dir, exist_ok=True)
        d = np.array(self.log)
        t = d[:, 0]
        g = d[:, 1:4]      # ground truth (x,y,theta) del punto p
        o = d[:, 4:7]      # odometria sola
        h = d[:, 7:10]     # KF
        qd = d[:, 10:12]
        Pd = d[:, 12:15]
        names = ['x [m]', 'y [m]', 'theta [rad]']

        def wrap(a):
            return np.arctan2(np.sin(a), np.cos(a))

        plt.figure(figsize=(7.5, 7.5))
        plt.plot(qd[:, 0], qd[:, 1], 'g--', lw=2, label='Deseada')
        plt.plot(g[:, 0], g[:, 1], 'b-', lw=1.6, label='Ground truth (Isaac)')
        plt.plot(o[:, 0], o[:, 1], color='orange', lw=1.2, label='Odometria sola (deriva)')
        plt.plot(h[:, 0], h[:, 1], 'r-', lw=1.1, label='Estimada KF (IMU+enc+LiDAR)')
        plt.axis('equal'); plt.grid(True, alpha=0.3); plt.legend(fontsize=9)
        plt.xlabel('x [m]'); plt.ylabel('y [m]')
        plt.title('Isaac Sim sim-to-real: KF vs odometria sola')
        plt.tight_layout()
        plt.savefig(os.path.join(self.out_dir, 'isaac_01_plano_xy.png'), dpi=130); plt.close()

        fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for i in range(3):
            gi, oi, hi = g[:, i], o[:, i], h[:, i]
            if i == 2:
                gi, oi, hi = np.unwrap(gi), np.unwrap(oi), np.unwrap(hi)
            ax[i].plot(t, gi, 'b-', lw=1.4, label='Ground truth')
            ax[i].plot(t, oi, color='orange', lw=1.0, label='Odometria sola')
            ax[i].plot(t, hi, 'r-', lw=1.0, label='Estimado KF')
            ax[i].set_ylabel(names[i]); ax[i].grid(True, alpha=0.3)
        ax[0].legend(loc='upper right', ncol=3); ax[2].set_xlabel('t [s]')
        fig.suptitle('Isaac Sim sim-to-real: real vs odometria(deriva) vs KF')
        fig.tight_layout()
        fig.savefig(os.path.join(self.out_dir, 'isaac_02_estados_tiempo.png'), dpi=130)
        plt.close(fig)

        err = g - h
        err[:, 2] = wrap(g[:, 2] - h[:, 2])
        two = 2.0 * np.sqrt(Pd)
        fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for i in range(3):
            ax[i].plot(t, err[:, i], 'k-', lw=1.0, label='Error GT-KF')
            ax[i].plot(t, two[:, i], 'r--', lw=1.0, label=r'$\pm2\sigma$')
            ax[i].plot(t, -two[:, i], 'r--', lw=1.0)
            ax[i].set_ylabel('err ' + names[i]); ax[i].grid(True, alpha=0.3)
        ax[0].legend(loc='upper right'); ax[2].set_xlabel('t [s]')
        fig.suptitle('Isaac Sim sim-to-real: error de estimacion del KF y covarianza')
        fig.tight_layout()
        fig.savefig(os.path.join(self.out_dir, 'isaac_03_error_estimacion.png'), dpi=130)
        plt.close(fig)

        def rmse(a, b):
            return float(np.sqrt(np.mean((a - b) ** 2)))

        def rmse_ang(a, b):
            return float(np.sqrt(np.mean(wrap(a - b) ** 2)))
        self.get_logger().info(
            f'RMSE vs GT  x: odo={rmse(o[:,0],g[:,0]):.3f} kf={rmse(h[:,0],g[:,0]):.3f} | '
            f'y: odo={rmse(o[:,1],g[:,1]):.3f} kf={rmse(h[:,1],g[:,1]):.3f} | '
            f'th: odo={rmse_ang(o[:,2],g[:,2]):.3f} kf={rmse_ang(h[:,2],g[:,2]):.3f}')


def main():
    rclpy.init()
    node = KFControlIsaac()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.saved:
            node.save_plots()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
