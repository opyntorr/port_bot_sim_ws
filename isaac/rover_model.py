"""
Modelo cinematico del Robot Diferencial Generalizado (paper Diaz & Kelly 2016),
versión para los nodos de Gazebo.

Para Gazebo conviene expresar el control en terminos del twist del chasis
(v_a, omega) en vez de velocidades de rueda, porque el JetAuto se comanda con
geometry_msgs/Twist por /cmd_vel. La equivalencia con el paper es exacta:

  Modelo 1 (ec. 7) para el punto p con offset h:
      q_dot_p = A(theta) [v_a, omega]^T
      A(theta) = [ cos(theta)  -h sin(theta) ;
                   sin(theta)   h cos(theta) ]
  det A = h  ->  invertible siempre que h != 0  (misma condicion del paper).

  Ley de control (ec. 33) en twist:
      [v_a, omega]^T = A(theta)^-1 [ qd_dot + Kp (qd - q_p) ]
  -> en lazo cerrado ideal  q_tilde_dot = -Kp q_tilde  (asintoticamente estable).

El punto p esta a distancia h del centro del eje de ruedas (base_footprint):
      q_p = [x + h cos(theta), y + h sin(theta)]
"""

import numpy as np


class RoverParams:
    """Parametros del modelo generalizado (en metros)."""

    def __init__(self, r=0.04825, h=0.10, d=0.194):
        self.r = r  # radio de rueda real del JetAuto [m]
        self.h = h  # offset del punto de interes p (h != 0) [m]
        self.d = d  # track diferencial equivalente [m]


def point_p(xi, h):
    """Postura del punto p a partir del estado del chasis xi=[x,y,theta]."""
    x, y, th = xi
    return np.array([x + h * np.cos(th), y + h * np.sin(th)])


def A_matrix(theta, h):
    """Matriz A(theta) del Modelo 1 (q_dot_p = A [v, w])."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -h * s],
                     [s,  h * c]])


def controller_twist(q_p, theta, qd, qd_dot, Kp, h):
    """
    Ley de control (ec. 33) en forma de twist. Devuelve (v_a, omega).
    Se evalua con el estado ESTIMADO (q_p y theta vienen del filtro de Kalman).
    """
    A = A_matrix(theta, h)
    q_tilde = qd - q_p
    vw = np.linalg.solve(A, qd_dot + Kp @ q_tilde)
    return float(vw[0]), float(vw[1])


# --------------------------------------------------------------------------- #
# Modelo de proceso del chasis (uniciclo) para la prediccion del KF.
# El punto a (base_footprint) sigue el modelo estandar accionado por (v, w).
# --------------------------------------------------------------------------- #
def f_unicycle(xi, v, w):
    """xi=[x,y,theta] -> xi_dot, accionado por el twist comandado (v, w)."""
    th = xi[2]
    return np.array([v * np.cos(th), v * np.sin(th), w])


def jacobian_unicycle(xi, v, w):
    """F = d f / d xi (3x3) del uniciclo, para linealizar el KF continuo."""
    th = xi[2]
    F = np.zeros((3, 3))
    F[0, 2] = -v * np.sin(th)
    F[1, 2] = v * np.cos(th)
    return F


# --------------------------------------------------------------------------- #
# Cinematica mecanum del JetAuto (para reconstruir el twist desde los encoders).
# IK (jetauto_chassis_sim.py):  fl=(vx-vy-k w)/r  fr=(vx+vy+k w)/r
#                               rl=(vx+vy-k w)/r  rr=(vx-vy+k w)/r
# FK (inversa) usada por el "sensor" de encoders:
# --------------------------------------------------------------------------- #
def mecanum_forward(w_fl, w_fr, w_rl, w_rr, r=0.04825, k=0.20):
    """4 velocidades de rueda [rad/s] -> twist del chasis (vx, vy, wz)."""
    vx = r * (w_fl + w_fr + w_rl + w_rr) / 4.0
    vy = r * (-w_fl + w_fr + w_rl - w_rr) / 4.0
    wz = r * (-w_fl + w_fr - w_rl + w_rr) / (4.0 * k)
    return vx, vy, wz
