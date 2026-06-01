# JetAuto — simulación sim-to-real (Ignition gz-sim)

Copia del proyecto con el **URDF del JetAuto** (en vez del `rosmaster_x3`), con **ruedas
mecanum de rodillos modelados que funcionan por fricción** y el stack de base replicado
del robot real (Jetson Orin `jetauto_ros2_ws` + Jetson Nano `jetauto_ws`).

## Cómo correr
```bash
# (en el entorno con ros-gz / ign gazebo — Docker o la Orin; esta laptop no los tiene)
colcon build --packages-select mi_proyecto_sim
source install/setup.bash
ros2 launch mi_proyecto_sim jetauto_sim.launch.py
#   use_rviz:=false use_teleop:=false   x:=.. y:=.. z:=.. yaw:=..
```

## Qué es fiel al robot real (sim-to-real)

| Aspecto | Real (Orin/Nano) | Sim |
|---|---|---|
| Geometría/mallas | `jetauto_description` (base 0.297×0.145×0.11255, ruedas r=0.049 en ±0.097397/±0.086125) | mismas mallas STL + mismas posiciones |
| Ruedas mecanum | físicas | **4 ruedas `continuous` + 12 rodillos esféricos c/u con fricción** (sin plugin cinemático) |
| Cinemática | `mecanum.py`: a=103, b=97, Ø=96.5 (r=0.04825), go=0.90, turn=0.93 | `jetauto_chassis_sim.py` con las **mismas constantes** |
| Comando | velocidad de rueda (no torque) | `velocity_controller` (ros2_control) — comanda velocidad |
| Odometría | dead-reckoning de cmd_vel (open-loop) `odom_raw` | idéntico (`odom_raw`, sin TF) |
| Fusión | madgwick(imu/data_raw→imu/data) + EKF(odom_raw+imu/data→odom+TF) | idéntico (`jetauto_ekf.yaml`, port del `ekf.yaml` real) |
| LiDAR | RPLIDAR A1 → `/scan` (`lidar_frame`) | gpu_lidar A1 → `/scan` (`lidar_frame`) |
| IMU | MPU-6050 → `/imu/data_raw` (`imu_link`) | sensor gz imu → `/imu/data_raw` (`imu_link`) |
| Cámara | Astra Pro (profundidad+RGB) | rgbd_camera gz → `/cam_1/*` (RGB en `/cam_1/image`) |
| TF | map→odom→base_footprint→base_link→…→lidar_frame | idéntico |

## Archivos (todos nuevos, no tocan el sim rosmaster existente)
- `urdf/jetauto/jetauto_sim.urdf.xacro` — robot top (ensambla todo)
- `urdf/jetauto/jetauto_base.urdf.xacro` — chasis + carcasas (geometría real)
- `urdf/jetauto/jetauto_mecanum_wheel.urdf.xacro` — rueda + 12 rodillos con fricción
- `urdf/jetauto/jetauto_sensors.urdf.xacro` — A1 + MPU-6050 + Astra (poses reales)
- `urdf/jetauto/jetauto_control.urdf.xacro` — ros2_control velocidad + ign_ros2_control
- `config/jetauto_controllers.yaml`, `config/jetauto_ekf.yaml`
- `mi_proyecto_sim/jetauto_chassis_sim.py` — driver fiel (cmd_vel→ruedas + odom_raw)
- `launch/jetauto_sim.launch.py`
- `meshes/jetauto/*.stl` — mallas reales (traídas de la Orin)
- `worlds/laberinto.sdf` — se le añadió el `imu-system` de gz

## Notas / pendientes de ajuste en runtime
- **Validado estáticamente** (xacro→URDF + `check_urdf` OK, cinemática y marcos verificados).
  Falta correrlo en gz (Docker/Orin) para afinar **fricción de rodillos** (`mu1/mu2/kp/kd`
  en `jetauto_mecanum_wheel.urdf.xacro`) si el strafe lateral patina o arrastra.
- La cámara de profundidad va en junta **fija** (en el real es un servo pan/tilt revolute).
- `a+b=0.20 m` es el valor **calibrado** de Hiwonder (la geometría pura da 0.1835);
  se usa el calibrado para que el comando de rueda sea idéntico al robot real.
