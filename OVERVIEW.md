# Overview de `port_bot_sim_ws`

> Resumen de alto nivel para entender rápido **qué hace** este repo y **cómo lo hace**.
> Para detalles de instalación/uso ver el `README.md`.

---

## 1. ¿Qué es esto en una frase?

Un **workspace de ROS 2 (Humble) + Gazebo Ignition (Fortress)** donde un **robot terrestre (AGV)** navega de forma **autónoma** por un laberinto, usando un **dron (UAV tipo Tello)** como "GPS visual" desde arriba. Es **solo simulación**; el código para el hardware real vive en el repo hermano `port_bot_ws`.

La idea central: el dron vuela primero, fotografía el laberinto desde arriba, construye un **mapa de ocupación** y localiza marcadores **ArUco** (robot y meta). Luego el robot terrestre usa ese mapa + correcciones del dron para planear una ruta y seguirla esquivando obstáculos.

---

## 2. Los dos actores

| Actor | Modelo | Rol |
|-------|--------|-----|
| **AGV (carrito)** | Yahboom Rosmaster X3 / JetAuto (tracción Mecanum) | Navega el laberinto en tierra |
| **UAV (dron)** | Tello (con "espejo" / cámara hacia abajo) | Mapeo aéreo + localización visual (ArUco) |

---

## 3. Pipeline de extremo a extremo

El flujo está orquestado por `src/mi_proyecto_sim/launch/simulacion.launch.py` y ocurre en **fases encadenadas con event handlers** (no todo arranca a la vez):

```
1. Arranca Gazebo + mundo (laberinto.sdf) + puente ROS<->Gazebo (ros_gz_bridge)
2. Spawn del carrito (jetauto_bringup) y del dron (tello)
3. MISIÓN DEL DRON (mision_dron.py):
     takeoff -> recorre grid de waypoints a ~2.2m -> foto en cada uno
     -> aterriza -> stitching de fotos -> binariza -> genera PGM/YAML del mapa
     -> detecta ArUco del carrito (id 4) y de la meta (id 0)
4. Al terminar la misión (OnProcessExit):
     - convierte el PGM a .pbstream (pgm_to_pbstream.py)
     - arranca SLAM (slam_toolbox o cartographer), map_server,
       publicador de TFs de ArUcos y el planificador RRT
5. NAVEGACIÓN del carrito (control_trayectoria.py):
     fusiona odometría + ArUco, sigue la ruta RRT, esquiva con LiDAR
     -> publica /cmd_vel
```

Punto clave de diseño: `simulacion.launch.py` **borra los artefactos viejos** (`mapa_mision.pgm/yaml`, `arucos.yaml`) al inicio, para que si la misión del dron falla no se reutilice un mapa obsoleto.

---

## 4. Nodos principales (paquete `mi_proyecto_sim`)

Este es el paquete propio y el corazón del proyecto. Sus nodos clave:

| Nodo | Archivo | Qué hace |
|------|---------|----------|
| **Misión del dron** | `mision_dron.py` | Vuela el grid de waypoints, toma fotos, hace stitching y genera el mapa de ocupación + detecta ArUcos |
| **Detector ArUco** | `detector_aruco.py` | Detecta marcadores en la cámara del dron y publica sus poses como **TFs estáticas** (localización "GPS visual") |
| **Control de trayectoria** | `control_trayectoria.py` | El "cerebro" del carrito: fusión de sensores + controlador **Kelly & Diaz** + evasión por **potenciales artificiales** + watchdog. Publica `/cmd_vel` |
| **Planificador RRT** | `src/planificador_rrt.cpp` (C++) | Genera la ruta (Rapidly-exploring Random Tree) sobre el mapa, con replanificación automática |
| **Filtro LiDAR** | `filtro_lidar.py` | Recorta el campo de visión del LiDAR a ~190° |
| **Publicador de TFs ArUco** | `publicador_tfs_arucos.py` | Lee `arucos.yaml` y publica las TFs que conectan el mapa del dron (`map_dron_origin`) con el frame `map` del SLAM |
| **Chasis simulado** | `jetauto_chassis_sim.py` | Lado ROS del control del carrito (consume `/cmd_vel`) |
| **Generador de mapa** | `generador_mapa.py` | Generación del occupancy grid desde la cámara del dron |

Otros nodos de apoyo: `odom_noise_filter.py` (ruido de odometría realista), `slam_occupancy_grid.py`, `amcl_localizer.py`, `planificador_astar.py` (alternativa a RRT, comentado), `Mcnamu_driver_PID_sim.py` (driver Mecanum).

---

## 5. Cómo se localiza el robot (la idea más importante)

Es una **fusión de 3 fuentes** con un **filtro pasa-baja (α≈0.1)**:

- **90 % Odometría de ruedas** → movimiento suave y continuo (pero acumula deriva).
- **10 % ArUco del dron** → corrige la deriva cada vez que el dron ve el marcador del carrito.
- **LiDAR** → tiene la "última palabra" para seguridad (evasión reactiva).

Detalles:
- **Snapshot persistente**: guarda la última calibración en `config/last_snapshot.json` para arrancar la navegación al instante aunque el dron no vea el marcador al inicio.
- **Watchdog (4 s)**: si el carrito no avanza en su ruta, pide automáticamente una ruta nueva al RRT.
- **Evasión (potenciales artificiales)**: frente (cono ±20°) frena a los 35 cm; lados (±95°) repulsión "explosiva" a 25 cm.

---

## 6. Control de posición del dron (paquete `tello_control_pos`)

Nodos que estabilizan/mueven el dron durante la misión (lanzados desde `simulacion.launch.py`):

- `optitrack_simulator` → simula un sistema de captura de movimiento (pose ground-truth con latencia/ruido).
- `pose_fuser_optitrack` → fusiona la pose del dron.
- `position_controller` → **PID** de posición y yaw que mueve el dron a cada waypoint.
- `plotter` → grafica trayectoria y reportes (`ultima_trayectoria_3d_tello.png`, etc.).

---

## 7. Mapeo y SLAM: dos backends intercambiables

El launch acepta el argumento `slam_backend`:

- **`slam_toolbox`** (default): SLAM online async usando `/scan_filtered`. Es el comportamiento original.
- **`cartographer`**: Cartographer en *localization mode* usando como prior **congelado** el `.pbstream` derivado del mapa del dron. El folder `tools/carto_protos/` contiene los protobuf generados de Cartographer y `tools/pgm_to_pbstream.py` hace la conversión PGM → pbstream.

En ambos casos el mapa del dron se publica en `/map_dron` (vía `nav2_map_server`) para no chocar con el `/map` que produce el SLAM del LiDAR.

---

## 8. Estructura del repositorio

```
port_bot_sim_ws/
├── src/
│   ├── mi_proyecto_sim/          ← PAQUETE PRINCIPAL (lógica propia)
│   │   ├── launch/               ← simulacion.launch.py (orquestador) + ~20 launch files
│   │   ├── mi_proyecto_sim/      ← nodos Python (control, dron, aruco, lidar...)
│   │   ├── src/                  ← planificador_rrt.cpp (C++)
│   │   ├── urdf/                 ← modelos del carrito (JetAuto / Mecanum)
│   │   ├── worlds/               ← laberinto.sdf (mundo Gazebo)
│   │   ├── config/               ← params SLAM, EKF, controladores, cámara, joystick
│   │   ├── maps/                 ← mapas generados (PGM/YAML/pbstream) + arucos.yaml
│   │   └── tools/                ← pgm_to_pbstream.py + protos de Cartographer
│   ├── tello_control_pos/        ← control de posición del dron (PID + optitrack sim)
│   ├── yahboom_rosmaster/        ← descripción/control/gazebo del Rosmaster X3 (vendor)
│   ├── yahboom_description/, yahboom_base/, yahboom_msgs/   ← drivers/msgs Yahboom (vendor)
│   └── yahboomcar_ydlidar/       ← driver del LiDAR YDLidar (vendor)
├── compose.yaml + Dockerfile     ← entorno containerizado (ROS 2 + Gazebo aislados)
├── README.md                     ← arquitectura + instrucciones de uso
└── (scripts varios .py de análisis: process_map.py, check_arucos.py, etc.)
```

Notas:
- `src/demo_tello_sim/` es un **git submodule** (modelos/plugins del Tello en Gazebo); puede no estar checkout-eado.
- El paquete propio es `mi_proyecto_sim`; los `yahboom_*` y `yahboomcar_ydlidar` son **dependencias vendorizadas** del robot/sensor.

---

## 9. Tecnologías y cómo lo corremos

- **ROS 2 Humble** + **Gazebo Ignition Fortress** (puente `ros_gz_bridge` para `/clock`, cámara del dron, `cmd_vel`).
- **Docker** (`compose.yaml` + `Dockerfile`): todo el stack corre aislado, con el código fuente montado por *volume bind* para desarrollo ágil.
- **OpenCV** (detección ArUco, stitching), **slam_toolbox / Cartographer**, **Nav2 map_server**, **robot_localization (EKF)**, **imu_filter_madgwick**.
- Visualización: **RViz2** (LiDAR, ruta RRT, pose) y **rqt_image_view** (cámaras con ArUco). Hay puentes web opcionales (rosbridge / foxglove / web_video_server) actualmente comentados.

Arranque típico (resumen):

```bash
xhost +local:root
docker compose up -d --build
docker exec -it entorno_robotica bash
colcon build --packages-select mi_proyecto_sim && source install/setup.bash
ros2 launch mi_proyecto_sim simulacion.launch.py
```
