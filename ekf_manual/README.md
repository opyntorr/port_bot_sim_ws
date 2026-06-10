# EKF manual (extendido) — JetAuto mecanum, robot real

Filtro de Kalman **extendido manual** (sin `robot_localization`) con **estado aumentado**
para estimar online el bias del giroscopio. Parametrizado con la caracterización REAL
del robot (gyro z: bias 0.199 °/s que deriva, ARW 1.062 °/√h → var 5e-6; LiDAR σ 4.3 mm).

## Estado y modelo
Estado 4D: `x = [px, py, θ, b_ω]` (pose global + bias del yaw-rate).

- **Predicción** (~50 Hz, modelo mecanum no lineal):
  - `vx, vy` de `/odom_raw` (dead-reckoning), `ω` de `/imu/data_raw` (giro crudo, con bias)
  - `θ̇ = ω_imu − b_ω` → se resta el bias estimado; `b_ω` es random walk
- **Corrección** con pose global `z=[x,y,θ]` (AMCL `/amcl_pose` o scan-match), `H=[I₃|0]`
  - la covarianza cruzada `P[θ,b_ω]` hace que el LiDAR **observe y aprenda** el bias

## Archivos
- `ekf_model.py` — núcleo matemático (f, Jacobianos F/G, Q/R, predict/update). `python3 ekf_model.py` corre el **autotest** (Jacobianos vs diferencias finitas + demo de convergencia del bias).
- `ekf_manual_node.py` — nodo ROS2 (rclpy, sin colcon). Subs `/odom_raw` + `/imu/data_raw` + `correction_topic`; pub `/odometry/ekf_manual`.
- `run_ekf.sh` — arranque en el Orin (dominio 0).

## Correr (en el Orin)
```bash
./run_ekf.sh
# o con parámetros:
./run_ekf.sh --ros-args -p correction_topic:=/amcl_pose -p publish_tf:=false \
             -p sig_b:=1e-4 -p sig_vx:=0.02 -p sig_vy:=0.02
```
La **corrección requiere un localizador global** (AMCL o slam_toolbox publicando pose);
sin correcciones el EKF corre solo-predicción y deriva como la odometría. Lanza AMCL
(`navegar_real_amcl.launch.py`) o adapta `correction_topic` a tu fuente de pose.

## Parámetros
| Param | Default | Qué es |
|---|---|---|
| `correction_topic` | `/amcl_pose` | pose global (PoseWithCovarianceStamped) |
| `publish_tf` | `false` | publica `map→base_footprint` (déjalo en false si corre AMCL) |
| `frame_id` / `child_frame` | `map` / `base_footprint` | marcos de salida |
| `sig_vx`, `sig_vy` | 0.02 m/s | ruido de la velocidad de odom |
| `sig_w` | 0 (→ ARW por dt) | ruido del giro |
| `sig_b` | 5e-4 | random walk del bias (↓ = bias más estable, más lento) |

## Validado
- `ekf_model.py`: Jacobianos F y G vs diferencias finitas **< 1e-5**; demo: error de rumbo a 120 s **0.06° (EKF) vs 24° (odometría sola)**, bias estimado 0.186° (real 0.199).
- Nodo end-to-end (publicador sintético, dominio aislado): fusiona los 3 topics y el bias estimado converge a **~0.2 °/s** siguiendo la trayectoria.

## Pendiente (F3/F4)
- F3: validar en piso que `b_ω` converge a ~0.0035 rad/s con AMCL real.
- F4: comparar `/odometry/ekf_manual` vs `robot_localization` vs OptiTrack (pipeline `~/evo_run.sh`); gráficas APE/RPE + estimación del bias.
