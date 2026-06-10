"""
Modelos de ruido de los sensores del JetAuto REAL  ->  unica fuente de verdad
para la inyeccion de ruido sim-to-real y para la sintonia (Q, R) del filtro.

Todo viene de caracterizaciones/ (medidas en el robot real):
  * IMU MPU-6050   -> imu/imu_summary.txt        (bias, ARW, var de medicion)
  * Encoders       -> motores/ENCODER_ODOMETRY.md (escala, cuantizacion, slip de yaw)
  * LiDAR MS200    -> lidar/lidar_summary.txt     (sigma(rango), dropout, espurios)
  * Envelope       -> README.md                   (vel/accel reales)

Idea sim-to-real: el MISMO numero que se usa para INYECTAR ruido en la simulacion
se usa para sintonizar el filtro de Kalman que luego lo corrige.
"""

import numpy as np

# =========================================================================== #
# IMU  —  MPU-6050, eje z (yaw rate).  /imu/data_raw @ 50 Hz
# =========================================================================== #
IMU_RATE_HZ = 50.0
IMU_GYRO_Z_BIAS = np.deg2rad(0.199)      # rad/s — bias residual que deriva en la sesion
IMU_GYRO_Z_VAR = 5.0e-6                   # (rad/s)^2 — cov. de medicion usada por el EKF real
IMU_GYRO_Z_ARW = 1.062                    # deg/sqrt(hr) — angle random walk (ruido blanco)
IMU_BIAS_DRIFT_RANGE = np.deg2rad(0.05)   # rad/s — rango de deriva del bias (~10 h)


def imu_arw_rate_std(dt):
    """
    Desv. estandar del RUIDO BLANCO del giro por muestra [rad/s], desde el ARW.
        ARW [deg/sqrt(hr)] -> N [rad/sqrt(s)] = deg2rad(ARW)/60   (60 = sqrt(3600 s))
        std_por_muestra = N / sqrt(dt)
    Verificacion: a 50 Hz da ~2.2e-3 rad/s -> var ~4.8e-6 ≈ IMU_GYRO_Z_VAR (5e-6). OK.
    """
    N = np.deg2rad(IMU_GYRO_Z_ARW) / 60.0
    return N / np.sqrt(dt)


def corrupt_imu_yaw_rate(true_w, rng, dt, bias=IMU_GYRO_Z_BIAS):
    """Inyecta el ruido real del giro: bias residual + ruido blanco (ARW)."""
    return true_w + bias + rng.normal(0.0, imu_arw_rate_std(dt))


# =========================================================================== #
# ENCODERS  —  odometria de ruedas mecanum.  @ 50 Hz
# =========================================================================== #
WHEEL_DIAMETER = 0.0965                   # m
WHEELBASE_A = 0.103                       # m
WHEELBASE_B = 0.097                       # m
WHEEL_K = WHEELBASE_A + WHEELBASE_B       # m (= lx+ly = 0.200)
WHEEL_RADIUS = WHEEL_DIAMETER / 2.0       # m
PPC = 4320                                # pulsos por ciclo
GO_FACTOR = 0.90
TURN_FACTOR = 0.93
ENC_LINEAR_CORRECTION = 1.085             # avance real 1.07 / odom 0.986 -> escala
ENC_YAW_UNDERESTIMATE = 0.88              # el giro por ruedas subestima ~12% (NO usar p/yaw)
ENC_QUANT_M = np.pi * WHEEL_DIAMETER / PPC  # m por cuenta (~7.0e-5)


def enc_linear_std(dt):
    """std del ruido (cuantizacion) de la velocidad lineal de encoders [m/s]."""
    quant_v = (ENC_QUANT_M / np.sqrt(12.0)) / dt   # cuantizacion uniforme -> q/sqrt(12)
    return float(np.hypot(quant_v, 0.002))         # + piso de ruido (slip/derrape)


def corrupt_encoder_forward(true_v, rng, dt):
    """
    Velocidad lineal medida por encoders. La parte lineal esta CALIBRADA
    (factor 1.085), asi que en media ~ true_v; queda el ruido de cuantizacion.
    (El yaw por ruedas NO se usa: subestima ~12% -> el yaw lo da la IMU.)
    """
    return true_v + rng.normal(0.0, enc_linear_std(dt))


# =========================================================================== #
# LiDAR  —  Orbbec MS200.  /scan @ 15 Hz
# =========================================================================== #
LIDAR_RATE_HZ = 15.0
LIDAR_MIN_RANGE = 0.05
LIDAR_MAX_RANGE = 8.0                     # margen confiable interiores (de 12 nominal)
LIDAR_QUANT_M = 0.001
LIDAR_DROPOUT = 0.043                     # 4.3% rayos sin retorno
LIDAR_SPURIOUS = 0.035                    # 3.5% retornos espurios
LIDAR_SIGMA_TYP = 0.0043                  # m, sigma de rango tipico

# curva sigma(rango) medida (puntos medios de cada bin de lidar_summary.txt)
_R_BINS = [0.25, 0.75, 1.25, 1.75, 2.25, 2.75, 3.25, 3.75]
_S_BINS = [0.0014, 0.0088, 0.0054, 0.0048, 0.0126, 0.0214, 0.0151, 0.0162]


def lidar_sigma(r):
    """sigma del rango [m] en funcion de la distancia (curva caracterizada)."""
    return np.maximum(np.interp(r, _R_BINS, _S_BINS), 0.0014)


def corrupt_lidar_range(true_r, rng, max_range=LIDAR_MAX_RANGE):
    """
    Inyecta el ruido real del MS200 a un rango verdadero:
      * dropout  -> sin retorno (max_range)
      * espurios -> rango aleatorio
      * normal   -> + ruido gaussiano sigma(r) + cuantizacion 1 mm
    """
    u = rng.random()
    if true_r >= max_range or u < LIDAR_DROPOUT:
        return max_range
    if u < LIDAR_DROPOUT + LIDAR_SPURIOUS:
        return rng.uniform(LIDAR_MIN_RANGE, max_range)
    r = true_r + rng.normal(0.0, float(lidar_sigma(true_r)))
    r = round(r / LIDAR_QUANT_M) * LIDAR_QUANT_M
    return min(max(r, LIDAR_MIN_RANGE), max_range)


# Pose absoluta derivada del LiDAR (scan-match / AMCL) = ancla del filtro.
# sigma escalado de forma conservadora desde el sigma de rango (robustez).
LIDAR_POSE_SIGMA_XY = 0.02                # m
LIDAR_POSE_SIGMA_YAW = np.deg2rad(1.0)    # rad


def corrupt_lidar_pose(true_xytheta, rng):
    """Pose absoluta (x,y,theta) tipo AMCL con el ruido del anclaje LiDAR."""
    n = rng.normal(0.0, 1.0, 3)
    return true_xytheta + np.array([
        LIDAR_POSE_SIGMA_XY * n[0],
        LIDAR_POSE_SIGMA_XY * n[1],
        LIDAR_POSE_SIGMA_YAW * n[2]])


# =========================================================================== #
# ENVELOPE de velocidad / aceleracion (chasis real)
# =========================================================================== #
V_MAX = 0.10        # m/s   (cap de navegacion)
W_MAX = 0.5         # rad/s
A_LIN = 1.3         # m/s^2
A_ANG = 3.4         # rad/s^2
A_ANG_DECEL = 4.5   # rad/s^2
