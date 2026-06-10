# Port del AGV (JetAuto mecanum) a Isaac Sim 4.5 — F0 / F1 + Tareas (Kalman + GridMap)

Sim **local** del JetAuto en Isaac Sim 4.5 (no robot real). El "cerebro" ROS2 (RViz, teleop,
y más adelante SLAM/RRT/misión) es el mismo del proyecto; Isaac solo reemplaza el front-end del
simulador honrando el **mismo contrato de tópicos** (`/clock`, `/odom`, TF, `/joint_states`,
`/cmd_vel`, `/scan`).

Las **dos tareas del curso** (Filtro de Kalman y Mapa de Ocupación probabilístico), que ya
corrían en Python puro y en Gazebo, están portadas aquí: mismo algoritmo (Kalman-Bucy +
control de seguimiento; log-odds + modelo inverso por haz), solo cambia la planta/sensores.
Ver **"Tareas 1 y 2"** abajo. Figuras de validación en `isaac/figs/`.

## Requisitos por terminal
En **cada** terminal (la de Isaac y la del cerebro): `source isaac/isaac_env.sh`
(fija `ROS_DOMAIN_ID=30` + RMW cyclonedds + `CYCLONEDDS_URI=cyclonedds_local.xml`).

> **DDS local (importante):** Isaac y el cerebro corren en la misma laptop. Con el cyclonedds
> *default* cada proceso elige una interfaz distinta (WiFi/docker/…) y **no se descubren**.
> Por eso `isaac_env.sh` fija `cyclonedds_local.xml` (loopback unicast) en **ambos** lados.
> Si cambias el env, **reinicia Isaac** (lee `CYCLONEDDS_URI` al arrancar). Si el CLI de ROS2
> se cuelga (`ros2 topic list` no devuelve), hay un **daemon viejo** con otra config:
> `ros2 daemon stop` y reintenta (o usa `--no-daemon`).

## F0 — Smoke test del bridge + /clock
```bash
source isaac/isaac_env.sh
$ISAACSIM/python.sh isaac/f0_clock_smoke.py           # (o --headless)
# otra terminal:
source isaac/isaac_env.sh
ros2 topic echo /clock        # debe avanzar
```

## F1 — Robot importado, conducible, /odom + TF + /joint_states
```bash
# 1) (solo si cambió el URDF) re-exportar el URDF plano desde el xacro:
#    ver "Re-exportar el URDF" abajo.
# 2) escena Isaac:
source isaac/isaac_env.sh
$ISAACSIM/python.sh isaac/scene_agv.py
# 3) cerebro + RViz + teleop (otra terminal):
source isaac/isaac_env.sh
ros2 launch isaac/jetauto_isaac_view.launch.py
```
Verificar: en RViz (frame fijo `odom`) se ve el robot y el TF; con **LB + sticks** avanza/strafe/gira;
`ros2 topic hz /odom` ≈ 50–60, `/joint_states` presente, `/clock` corriendo.

## Tareas 1 y 2 (Kalman + GridMap) — replica de Gazebo en Isaac

Los nodos del "cerebro" corren en el **host** (ROS2 Humble), NO en el docker `integration`
(ese está aislado en dominio 42; Isaac usa dominio 30 + cyclonedds loopback). Por eso **cada
terminal** necesita `source /opt/ros/humble/setup.bash` **y** `source isaac/isaac_env.sh`.

### Tarea 1 — Filtro de Kalman (`kf_control_isaac.py`)
KF continuo-discreto + control de seguimiento (Modelo 1 de Kelly, offset `h`). Predicción
propioceptiva con ruido caracterizado del robot real (encoders + IMU), corrección con ancla
LiDAR tipo AMCL; el `/odom` de Isaac es el ground truth.
```bash
# terminal A — escena (mundo vacío):
source isaac/isaac_env.sh
$ISAACSIM/python.sh isaac/scene_agv.py            # (quita --headless ya implícito; añade --headless para sin ventana)
# terminal B — nodo KF (conduce el círculo y guarda figuras):
./isaac/run_kalman.sh                              # = source envs + python3 kf_control_isaac.py
```
Tras ~90 s de sim guarda `isaac_01_plano_xy.png`, `isaac_02_estados_tiempo.png`,
`isaac_03_error_estimacion.png`. Resultado típico: RMSE del KF ~0.007–0.009 (x,y[m], θ[rad])
vs odometría sola que **deriva** (hasta ~0.8 rad en θ) — el ancla LiDAR la acota.

### Tarea 2 — Mapa de ocupación (`gridmap_isaac.py`)
Rejilla log-odds desde el **RTX lidar 2D** (config `RPLIDAR_S2E`, near 0.05 m ≈ MS200) montado
en `lidar_frame`. La escena `scene_gridmap.py` añade un cuarto 6×6 con 4 paredes + 4 cajas
(misma geometría que `tareas_room.sdf` de Gazebo). El árbol TF `base_footprint→lidar_frame` lo
da `robot_state_publisher`; la escena publica `odom→base_footprint`.
```bash
# terminal A — escena (cuarto + RTX lidar):
source isaac/isaac_env.sh
$ISAACSIM/python.sh isaac/scene_gridmap.py        # (añade --headless para sin ventana)
# terminal B — robot_state_publisher + gridmap + KF (driver), todo junto:
./isaac/run_gridmap.sh
```
Autoguarda `isaac_10_mapa_ocupacion.png` cada ~10 s. En RViz: display **Map** en
`/mapa_probabilistico` (frame `odom`) para verlo construirse en vivo.

### Notas del port (por qué difiere del de Gazebo)
- **Encoders desde `/odom.twist`, no de las ruedas:** la base es cinemática (teleport), las
  ruedas son pasivas → `/joint_states` no da velocidad útil. La fuente propioceptiva se toma del
  twist del simulador (verificado: está en **marco cuerpo**), y se le inyecta el ruido de encoders.
- **GridMap submuestrea a ~400 haces:** el RTX lidar da 3200 pts/360° a 55 Hz; el Bresenham en
  Python no alcanza ese ritmo y la cola atrasaba la pose (mapa girado). Con ~400 haces (resolución
  tipo MS200) el callback sigue el ritmo y el mapa sale alineado a ejes.
- **TF en Isaac:** el stamp del `/scan` del RTX writer está en otra época que el `/clock` de sim
  (no se puede usar para el lookup); por eso el nodo usa la última TF disponible, que con el
  callback rápido coincide con el barrido.

> **Diagnóstico del "no se movía":** la escena conduce bien; lo que faltaba era **publicar
> `/cmd_vel`** en el dominio 30 (teleop o un nodo, con `isaac_env.sh` sourceado). Además el
> `print("[F1] Listo")` queda en buffer, así que la consola parece colgada aunque ya esté corriendo
> (comprobar con `ros2 topic hz /clock`).

## Re-exportar el URDF plano (assets/jetauto.urdf)
`mi_proyecto_sim` no está instalado, así que usamos un overlay de ament para que `$(find)` resuelva:
```bash
SRC=~/agv_uav_project_jetauto/src/mi_proyecto_sim
OV=~/agv_uav_project_jetauto/isaac/.ament_overlay
mkdir -p "$OV/share/ament_index/resource_index/packages" ~/agv_uav_project_jetauto/isaac/assets
: > "$OV/share/ament_index/resource_index/packages/mi_proyecto_sim"
ln -sfn "$SRC" "$OV/share/mi_proyecto_sim"
source /opt/ros/humble/setup.bash
AMENT_PREFIX_PATH="$OV:$AMENT_PREFIX_PATH" \
  xacro "$SRC/urdf/jetauto/jetauto_sim.urdf.xacro" use_aruco:=false \
  -o ~/agv_uav_project_jetauto/isaac/assets/jetauto.urdf
sed -i "s#$OV/share/mi_proyecto_sim#$SRC#g" ~/agv_uav_project_jetauto/isaac/assets/jetauto.urdf
```

## Decisiones / notas
- **Mecanum = base holonómica cinemática**: se conduce la base por override de velocidad del root
  (vx, vy, wz) desde `/cmd_vel`; los 48 rodillos no se usan para el strafe (más robusto que la
  física de rodillos en PhysX). Upgrade opcional: física fiel de rodillos.
- **Odometría = ground-truth** del simulador (`IsaacComputeOdometry` → `/odom` + TF), igual que la
  sim de Gazebo del proyecto.
- Si `/odom` sale vacío, cambiar `chassis_prim` en `scene_agv.py` de `/jetauto/base_footprint` a
  `/jetauto/base_link`. (Verificado: con el default sale poblado.)
- **RTX lidar (F2, hecho para gridmap):** `scene_gridmap.py` crea el sensor con
  `IsaacSensorCreateRtxLidar` (config `RPLIDAR_S2E`) parentado a `lidar_frame`, un render product, y
  el writer `RtxLidar` `ROS2PublishLaserScan` (`/scan`, frame `lidar_frame`).
- Roadmap: F2 resto de sensores (IMU, cámara), F3 mundo completo (laberinto+ArUco), F4 SLAM/RRT/nav,
  F5 Tello, F6 orquestación.
