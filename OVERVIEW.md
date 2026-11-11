# Technical Overview: `port_bot_sim_ws`

A high-level architectural summary explaining what this workspace accomplishes, how its subsystems interact, and how it bridges simulation and physical deployment.

For comprehensive mathematical derivations, benchmarks, and deployment steps, refer to [README.md](file:///home/opyntorr/port_bot_sim_ws/README.md).

---

## 1. Executive Summary

`port_bot_sim_ws` is an integrated **ROS 2 Humble Sim-to-Real workspace** hosting an autonomous multi-robot cooperative logistics system designed for **GNSS/GPS-denied maritime seaport environments**.

The platform orchestrates:
- An **Unmanned Aerial Vehicle (UAV, DJI Tello)** executing an autonomous aerial lawnmower survey, synthesizing a metric orthomosaic through 3-phase pose-based stitching, and identifying container destination targets using ArUco visual markers.
- An **Autonomous Mobile Ground Robot (AMR, Hiwonder JetAuto / Yahboom Rosmaster X3 with 4WD Mecanum drive)** receiving target spatial goals via dynamic ROS 2 TFs, navigating through narrow port aisles using global search planners (Theta* / RRT) and local tracking (Kelly & Diaz), with active LiDAR safety repulsions.
- A **From-Scratch Continuous-Discrete Kalman Filter (Kalman-Bucy formulation)** fusing wheel odometry, IMU angular velocity, and absolute global observations (OptiTrack NatNet in real runs, or AMCL / ArUco in simulation) while estimating dynamic gyro bias $b_\omega$, yielding a **~79x heading drift reduction in Isaac Sim** and **24x in Gazebo**.
- A **Distributed Dual-Jetson Architecture** where a Jetson Nano (Docker) exposes low-level sensors and motor controllers, communicating via CycloneDDS Unicast with Chrony PTP time synchronization (<5 ms jitter) to a Jetson Orin running deep visual SLAM (ORB-SAM-E), SLAM Toolbox, and high-level mission intelligence.

---

## 2. The Two Cooperative Actors

| Actor | Platform | Physical & Simulated Capabilities |
|---|---|---|
| **Ground AMR** | Hiwonder JetAuto / Yahboom Rosmaster X3 | 4WD Mecanum omnidirectional drive, Oradar MS200 2D LiDAR (190° FOV clipping), MPU-6050 6-DOF IMU, Orbbec Astra RGB-D camera. Performs autonomous trajectory following and reactive obstacle avoidance. |
| **Aerial UAV** | DJI Tello Quadrotor | Downward optical mirror, 720p nadir streaming, 100 Hz PID position controller, OptiTrack + odometry pose fusion (`pose_fuser`), and autonomous flight state machine. |

---

## 3. End-to-End Mission Pipeline

The mission flow is orchestrated by `src/mi_proyecto_sim/launch/simulacion.launch.py` (and corresponding physical launches) utilizing sequential ROS 2 event handlers:

```
[Phase 1: Environment Initialization]
  ├── Gazebo Fortress / Isaac Sim world loads (SolidWorks port terminal model)
  └── Ground AMR and Aerial UAV spawn at calibrated home datums

[Phase 2: Aerial Survey & Perception]
  ├── UAV takes off autonomously under 100 Hz PID control
  ├── Traverses lawnmower waypoint grid at altitude h = 2.2 m
  ├── Captures 16 nadir photographs with synchronized pose timestamps
  └── Executes controlled touchdown at designated landing pad

[Phase 3: 3-Phase Orthomosaic Synthesis & Spatial Handoff]
  ├── Phase A: Initial pose-based placement
  ├── Phase B: FFT phase correlation on Canny edge maps (sub-pixel alignment)
  ├── Phase C: Global L-BFGS non-linear optimization + multiband blending
  ├── Generates 500 px/m ROS 2 Occupancy Grid (.pgm / .yaml) with 96.8% agreement
  ├── Detects ArUco marker ID 4 (AMR initial pose) and ID 0 (Container Goal)
  └── Dynamic TF Broadcaster publishes: map -> map_dron_origin -> meta_aruco

[Phase 4: Autonomous Ground Navigation]
  ├── publicador_tfs_arucos emits /alignment_ready event latch
  ├── SLAM (SLAM Toolbox / Cartographer in localization mode) loads occupancy prior
  ├── C++ Theta* / RRT planner computes collision-free trajectory (<100 ms)
  └── Kelly & Diaz carrot-follower executes omnidirectional path tracking

[Phase 5: Continuous Fusion & Active Safety]
  ├── Continuous-Discrete Kalman Filter bounds odometry drift using IMU gyro bias
  └── Oradar MS200 LiDAR enforces 3-cone potential field safety:
        - Front Cone (±20°): 35 cm proportional braking
        - Side Cones (±95°): 25 cm explosive repulsive steering
```

---

## 4. Key Packages & Subsystem Inventory

### Core Integration Package (`src/mi_proyecto_sim`)
- `mision_dron.py`: Autonomous UAV flight state machine coordinating waypoints and photographic capture.
- `mision/stitch_pose.py`: Master orchestrator for the 3-phase orthomosaic stitching pipeline.
- `detector_aruco.py`: High-precision ArUco marker detector with snapshot persistence (`config/last_snapshot.json`).
- `publicador_tfs_arucos.py`: Broadcasts static and dynamic coordinate transformations linking aerial coordinate frames to ground SLAM datums.
- `control_trayectoria.py`: Ground AMR motion controller combining Kelly & Diaz carrot following, sensor fusion, 3-cone potential field repulsions, and a 4-second stall watchdog.
- `src/planificador_rrt.cpp`: High-performance C++ implementation of RRT and any-angle Theta* path planning over occupancy costmaps.
- `filtro_lidar.py`: Real-time 2D laser scan filtering, clipping spurious back-reflections to an active 190° field of view.
- `generador_mapa.py`: Direct occupancy grid generation from raw aerial imagery.

### Dedicated UAV Control Subsystem (`src/demo_tello_sim`)
- Submodule repository: [`demo_tello_sim`](https://github.com/opyntorr/demo_tello_sim) (Branch: `fix/tello-takeoff-deadlock`).
- `tello_node.py`: Driver handling UDP socket serialization, mutex locks preventing command desynchronization, and TOF altitude fallbacks.
- `controller.py`: 100 Hz position PID controller ($k_p=0.5, k_i=0.06, k_d=0.35$) with geofencing gates.
- `pose_fuser.py`: Extrapolation and sensor fusion uniting OptiTrack ground truth with internal visual odometry, equipped with 180° flip rejection and a 500 ms safety watchdog.
- `test_flight_gate.py`: 11 contract test suites verifying flight gates, emergency hover, and landing behaviors.

### Deep Visual SLAM Subsystem (`ros2_orb_slam3`)
- Monocular ORB-SLAM3 enhanced with Segment Anything **EfficientSAM3** (RepViT + MobileCLIP backbone).
- Deployed on Jetson Orin with CUDA/TensorRT acceleration.
- Scrubs dynamic features corresponding to human dock workers and transient vehicles at ~1.67 FPS, preserving long-term landmark map integrity.

### Vendor & Hardware Drivers
- `yahboom_rosmaster/`: Robot URDF descriptions, Mecanum drive physics plugins, and controller configurations.
- `yahboomcar_ydlidar/`: ROS 2 driver nodes for Oradar MS200 / YDLidar laser scanners.
- `SunriseRobotLib.py` & `Mcnamu_driver_PID.py`: Physical I2C motor control library interfacing with low-level STM32/driver hardware.

---

## 5. Sim-to-Real Modularity

The system enforces strict interface parity across all four stages of development:

```
[Tier 1: Python Model] 
       │
       ▼
[Tier 2: Gazebo Fortress] (SDF Worlds, ros_gz_bridge, clipping)
       │
       ▼
[Tier 3: NVIDIA Isaac Sim 4.5] (PhysX contact dynamics, 40-roller slip, RTX perception)
       │
       ▼
[Tier 4: Physical Hardware] (JetAuto 4WD, DJI Tello, Dual Jetson Orin/Nano, OptiTrack)
```

- **Topic & TF Parity**: Every algorithm consumes and publishes identical message types (`geometry_msgs/Twist`, `sensor_msgs/LaserScan`, `nav_msgs/Odometry`, `tf2_msgs/TFMessage`) regardless of whether the backend is Gazebo, Isaac Sim, or the physical Jetson bridge.
- **DDS Domain Isolation**:
  - `ROS_DOMAIN_ID=42`: Simulation environment.
  - `ROS_DOMAIN_ID=0`: Physical robot deployment.
  - `ROS_DOMAIN_ID=30`: OptiTrack NatNet motion capture tracking.
- **Clock Determinism**: In physical distributed compute, Chrony PTP time synchronization locks Jetson Nano and Jetson Orin clocks to within $<5$ ms jitter over point-to-point Gigabit Ethernet.
