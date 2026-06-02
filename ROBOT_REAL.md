# JetAuto real — pipeline mapeo / guardado / navegación (bridge Orin + Nano)

Este repo contiene tanto la **simulación** (Gazebo Ignition) como el port al **robot real
JetAuto**. Este documento cubre el robot real.

## Arquitectura (bridge)

- **Jetson Nano** (Docker, `192.168.5.2`): hardware. Publica `/scan`, `/odom_raw`,
  `/imu/data_raw`, `/cam_1/image`; consume `/cmd_vel`.
- **Jetson Orin** (`192.168.5.1`, hotspot `10.42.1.1`): cómputo. `orin_compute.launch.py`
  (RSP + EKF → `/odom` + TF) corre como `jetauto-orin.service`. Encima va el SLAM y el
  "cerebro" (`mi_proyecto_sim`).
- **Laptop**: teleop (control), RViz, y lanza el cerebro en el Orin por SSH.
- Comunicación: **CycloneDDS** sobre el ethernet Orin↔Nano; la laptop entra al dominio 0
  con `~/cyclonedds-laptop.xml` (peers unicast, `AllowMulticast=false`).

## Paquetes

| Paquete | Dónde corre | Qué hace |
|---|---|---|
| `src/jetauto_teleop` | laptop | teleop del control Xbox/8BitDo → `/cmd_vel` |
| `src/jetauto_rviz` | laptop | RViz + launches todo-en-uno + guardado de mapas |
| `src/mi_proyecto_sim` | Orin | cerebro: SLAM, filtro_lidar, planificador_rrt, nav_goal_bridge, control_diferencial. Los launches `*_bridge` son la versión real (sin Gazebo). |
| `orin/` | Orin | scripts de arranque que viven en `~` del Orin (referencia/backup) |

## Flujo de uso (todo desde la laptop)

Precondición: contenedor del Nano + `jetauto-orin.service` corriendo (hardware + EKF).

### 1. Mapear (con el control)
```bash
ros2 launch jetauto_rviz mapear_real.launch.py
```
Arranca teleop + RViz + el SLAM en el Orin (por SSH). Maneja con el control (mantén LB).

### 2. Guardar el mapa (sin SSH)
En otra terminal de la laptop, mientras mapeas:
```bash
ros2 launch jetauto_rviz guardar_mapa.launch.py map_name:=mi_mapa
```
Guarda en `~/jetauto_maps/` (laptop) **y** sube copia al Orin `~/maps/` por scp.
Sin `map_name` usa timestamp.

### 3. Navegar (cargar mapa + goal)
```bash
ros2 launch jetauto_rviz navegar_real.launch.py map:=/home/jetson/maps/mi_mapa.yaml
```
En RViz:
1. **2D Pose Estimate** → ancla la pose inicial del robot en el mapa (emula el aruco del
   carrito; congela la alineación `map → map_dron_origin`).
2. **2D Goal Pose** → destino.
3. `planificador_rrt` calcula la ruta (verde), `control_diferencial` la ejecuta. El SLAM
   sigue rescaneando en vivo (obstáculos dinámicos).

## Notas / gotchas

- **Regla de oro:** NO lanzar `view.launch.py slam:=true` junto con `mapear_real`/`navegar_real`
  (ambos traen su propio slam → conflicto de TF `map→odom`).
- El SLAM usa el config validado de `jetauto_slam` (`/scan` directo, 12 m), no el de sim.
- `filtro_lidar` (`/scan_filtered`) solo alimenta la evasión del control, no el SLAM.
- Los scripts en `orin/` son copias de referencia; viven en `~` del Orin
  (`orin_brain_start.sh` lanza los launches del cerebro con el entorno DDS correcto).
- El cerebro depende del workspace del bridge en el Orin (`~/jetauto_ros2_ws`:
  `jetauto_bringup`, `jetauto_slam`, `jetauto_controller`), que NO está en este repo.
