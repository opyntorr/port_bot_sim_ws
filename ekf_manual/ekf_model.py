#!/usr/bin/env python3
"""
EKF manual (extendido) para el JetAuto mecanum  —  NUCLEO MATEMATICO (sin ROS).

Estado aumentado (4D):   x = [px, py, theta, b_w]
    px, py  : posicion en el marco global (map) [m]
    theta   : rumbo [rad]
    b_w     : bias del yaw-rate del giroscopio [rad/s]  <- se ESTIMA online

Entrada (propioceptiva):  u = [vx, vy, w_imu]
    vx, vy  : velocidades de cuerpo (FK mecanum / odom dead-reckoning) [m/s]
    w_imu   : yaw-rate CRUDO del giroscopio (con bias) [rad/s]

Modelo de movimiento (no lineal -> "extendido"):
    px+   = px + (vx*cos(th) - vy*sin(th)) * dt
    py+   = py + (vx*sin(th) + vy*cos(th)) * dt
    th+   = th + (w_imu - b_w) * dt        # se RESTA el bias estimado
    b_w+  = b_w                            # random walk (lo mueve Q)

Medicion (exteroceptiva, p.ej. AMCL/scan-match):  z = [px, py, theta]
    h(x) = [px, py, theta]   ->   H = [I3 | 0]  (la pose NO observa b_w directo,
    pero la covarianza cruzada P[theta,b_w] hace que el LiDAR "aprenda" el bias).

Constantes de ruido = caracterizacion REAL del robot (sensor_models.py):
    IMU gyro z: bias 0.199 deg/s (DERIVA), ARW 1.062 deg/sqrt(hr) -> var ~5e-6 (rad/s)^2
    LiDAR MS200: sigma_range 4.3 mm  ->  sigma_pose_xy/yaw para la correccion
"""

import numpy as np

# --------------------------------------------------------------------------- #
# Constantes de ruido derivadas de la caracterizacion (rad, s, m).
# Son DEFAULTS; el nodo las expone como parametros ROS para re-sintonizar.
# --------------------------------------------------------------------------- #
IMU_GYRO_Z_ARW_DEG = 1.062          # deg/sqrt(hr)
IMU_GYRO_Z_BIAS = np.deg2rad(0.199)  # rad/s (residual que deriva)


def imu_arw_rate_std(dt):
    """std del ruido blanco del giro por muestra [rad/s] desde el ARW.
    N [rad/sqrt(s)] = deg2rad(ARW)/60 ; std = N/sqrt(dt). A 50 Hz ~2.2e-3."""
    N = np.deg2rad(IMU_GYRO_Z_ARW_DEG) / 60.0
    return N / np.sqrt(max(dt, 1e-6))


def wrap(a):
    """Envuelve un angulo a [-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


# --------------------------------------------------------------------------- #
# Modelo de movimiento y sus Jacobianos
# --------------------------------------------------------------------------- #
def f_motion(x, u, dt):
    """Propaga el estado un paso dt con la entrada u. Devuelve x+ (4,)."""
    px, py, th, bw = x
    vx, vy, w_imu = u
    c, s = np.cos(th), np.sin(th)
    return np.array([
        px + (vx * c - vy * s) * dt,
        py + (vx * s + vy * c) * dt,
        th + (w_imu - bw) * dt,
        bw,
    ])


def jacobian_F(x, u, dt):
    """Jacobiano de f respecto al ESTADO (4x4)."""
    _, _, th, _ = x
    vx, vy, _ = u
    c, s = np.cos(th), np.sin(th)
    F = np.eye(4)
    F[0, 2] = (-vx * s - vy * c) * dt
    F[1, 2] = (vx * c - vy * s) * dt
    F[2, 3] = -dt
    return F


def jacobian_G(x, u, dt):
    """Jacobiano de f respecto a la ENTRADA u=[vx,vy,w_imu] (4x3).
    Mapea el ruido de los sensores propioceptivos al estado."""
    _, _, th, _ = x
    c, s = np.cos(th), np.sin(th)
    G = np.zeros((4, 3))
    G[0, 0] = c * dt
    G[0, 1] = -s * dt
    G[1, 0] = s * dt
    G[1, 1] = c * dt
    G[2, 2] = dt
    return G


def build_Q(x, u, dt, sig_vx, sig_vy, sig_w, sig_b):
    """Covarianza de proceso Q (4x4).
    - ruido de vx,vy,w_imu propagado por G (caracterizacion)
    - random walk del bias de giro en el termino [3,3] (sig_b^2 * dt)
    """
    G = jacobian_G(x, u, dt)
    Qu = np.diag([sig_vx ** 2, sig_vy ** 2, sig_w ** 2])
    Q = G @ Qu @ G.T
    Q[3, 3] += sig_b ** 2 * dt
    return Q


# --------------------------------------------------------------------------- #
# Pasos del EKF (funciones puras: reciben/devuelven x, P)
# --------------------------------------------------------------------------- #
def ekf_predict(x, P, u, dt, sig_vx, sig_vy, sig_w, sig_b):
    F = jacobian_F(x, u, dt)
    Q = build_Q(x, u, dt, sig_vx, sig_vy, sig_w, sig_b)
    x_pred = f_motion(x, u, dt)
    x_pred[2] = wrap(x_pred[2])
    P_pred = F @ P @ F.T + Q
    P_pred = 0.5 * (P_pred + P_pred.T)   # simetriza
    return x_pred, P_pred


# Medicion de pose: H = [I3 | 0]
H_POSE = np.array([[1.0, 0, 0, 0],
                   [0, 1.0, 0, 0],
                   [0, 0, 1.0, 0]])


def ekf_update_pose(x, P, z, R):
    """Correccion con una pose [px,py,theta] (AMCL/scan-match). z, R 3x3."""
    H = H_POSE
    innov = z - H @ x
    innov[2] = wrap(innov[2])            # angulo envuelto
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x_new = x + K @ innov
    x_new[2] = wrap(x_new[2])
    P_new = (np.eye(4) - K @ H) @ P
    P_new = 0.5 * (P_new + P_new.T)
    return x_new, P_new


# --------------------------------------------------------------------------- #
# Autotest: verifica los Jacobianos contra diferencias finitas
# --------------------------------------------------------------------------- #
def _finite_diff(func, v, eps=1e-6):
    n = len(v)
    out0 = func(v)
    J = np.zeros((len(out0), n))
    for i in range(n):
        dv = v.copy(); dv[i] += eps
        J[:, i] = (func(dv) - out0) / eps
    return J


if __name__ == "__main__":
    rng = np.random.default_rng(0)
    dt = 1.0 / 50.0
    ok = True
    for _ in range(200):
        x = np.array([rng.uniform(-5, 5), rng.uniform(-5, 5),
                      rng.uniform(-np.pi, np.pi), rng.uniform(-0.05, 0.05)])
        u = np.array([rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2),
                      rng.uniform(-0.5, 0.5)])
        F_an = jacobian_F(x, u, dt)
        F_fd = _finite_diff(lambda xx: f_motion(xx, u, dt), x)
        G_an = jacobian_G(x, u, dt)
        G_fd = _finite_diff(lambda uu: f_motion(x, uu, dt), u)
        eF = np.max(np.abs(F_an - F_fd))
        eG = np.max(np.abs(G_an - G_fd))
        if eF > 1e-5 or eG > 1e-5:
            ok = False
            print(f"FALLO  errF={eF:.2e}  errG={eG:.2e}")
            break
    print("Jacobianos F y G vs diferencias finitas:",
          "OK (<1e-5)" if ok else "FALLARON")

    # demo: 120 s con ground-truth sintetico. El giro REAL es constante; la IMU
    # lo mide con bias+ruido; el LiDAR/AMCL corrige ~2 Hz. El EKF debe ESTIMAR el bias.
    true_bias = np.deg2rad(0.199)
    sig_vx = sig_vy = 0.02; sig_w = imu_arw_rate_std(dt); sig_b = 5e-4
    R = np.diag([0.02 ** 2, 0.02 ** 2, 0.03 ** 2])
    xt = np.zeros(4)                       # ESTADO REAL (bias=0 por definicion)
    x = np.zeros(4); P = np.diag([0.04, 0.04, 0.04, (np.deg2rad(1.0)) ** 2])
    err_th_odo = err_th_kf = None; th_odo = 0.0
    for k in range(int(120 / dt)):
        tvx, tvy, tw = 0.15, 0.0, 0.3
        xt = f_motion(xt, np.array([tvx, tvy, tw]), dt)          # verdad
        # sensores con ruido REAL
        u = np.array([tvx + rng.normal(0, sig_vx), tvy + rng.normal(0, sig_vy),
                      tw + true_bias + rng.normal(0, sig_w)])
        th_odo += (u[2]) * dt                                    # odometria SIN corregir bias
        x, P = ekf_predict(x, P, u, dt, sig_vx, sig_vy, sig_w, sig_b)
        if k % 25 == 0:                                          # AMCL/LiDAR ~2 Hz
            z = xt[:3] + rng.normal(0, [0.02, 0.02, 0.03])
            x, P = ekf_update_pose(x, P, z, R)
    err_th_odo = abs(wrap(th_odo - xt[2]))
    err_th_kf = abs(wrap(x[2] - xt[2]))
    print(f"bias estimado final = {np.rad2deg(x[3]):.3f} deg/s (real {np.rad2deg(true_bias):.3f})")
    print(f"error de rumbo @120s:  odometria sola = {np.rad2deg(err_th_odo):.2f} deg  "
          f"|  EKF = {np.rad2deg(err_th_kf):.2f} deg")
