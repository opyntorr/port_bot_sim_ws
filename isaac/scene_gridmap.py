#!/usr/bin/env python3
# TAREA 2 — Escena AGV (JetAuto) en Isaac Sim 4.5 para el MAPA DE OCUPACION.
#
# Igual que scene_agv.py (importa el URDF, conduce la base de forma holonomica
# cinematica desde /cmd_vel, publica /clock /odom TF /joint_states) PERO ademas:
#   * arma un CUARTO 6x6 con paredes y cajas (misma geometria que tareas_room.sdf
#     de Gazebo), como prims renderizables que el lidar puede "ver",
#   * monta un RTX lidar 2D (config RPLIDAR_S2E, near 0.05 m ~ MS200) en el
#     frame lidar_frame del robot y publica /scan (sensor_msgs/LaserScan).
#
# El mapeo lo hace gridmap_node.py (host) con /scan + TF odom->lidar_frame; el
# robot lo mueve kf_control_isaac.py (circulo). El arbol TF base->lidar lo da
# robot_state_publisher (host) con assets/jetauto.urdf; la escena solo publica
# odom->base_footprint.
#
# Correr:   source isaac/isaac_env.sh
#           $ISAACSIM/python.sh isaac/scene_gridmap.py            (con ventana)
#           $ISAACSIM/python.sh isaac/scene_gridmap.py --headless
import argparse
import math
import os

from isaacsim import SimulationApp

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "assets", "jetauto.urdf")
# config de lidar 2D (Slamtec RPLIDAR S2E): 360 deg, near 0.05 m, far 30 m
LIDAR_CONFIG = "RPLIDAR_S2E"

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--urdf", default=URDF_PATH)
parser.add_argument("--x", type=float, default=-1.0)          # spawn = igual que Gazebo
parser.add_argument("--y", type=float, default=-1.0)
parser.add_argument("--z", type=float, default=0.06)
parser.add_argument("--yaw", type=float, default=math.pi / 2)  # 1.5708 rad (rumbo +y)
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": args.headless})

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.replicator.core as rep
import omni.timeline
import usdrt.Sdf
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
enable_extension("isaacsim.sensors.rtx")
simulation_app.update()

# --- 1) Importar el URDF ---------------------------------------------------
_, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = False
import_config.distance_scale = 1.0
_, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=args.urdf,
    import_config=import_config,
    get_articulation_root=True,
)
print(f"[F3] URDF importado en prim: {prim_path}")

stage = omni.usd.get_context().get_stage()

# --- 1b) Materiales: verde -> aluminio anodizado, negro -> plastico mate -------
# (el importador de URDF deja el robot blanco; ver jetauto_materials.py)
from jetauto_materials import apply_jetauto_materials
apply_jetauto_materials(stage, prim_path)

# --- 2) Fisica + piso + luz ------------------------------------------------
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(0.0)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
physx = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
physx.CreateEnableCCDAttr(True)
physx.CreateEnableStabilizationAttr(True)
physx.CreateEnableGPUDynamicsAttr(False)
physx.CreateBroadphaseTypeAttr("MBP")
physx.CreateSolverTypeAttr("TGS")

omni.kit.commands.execute(
    "AddGroundPlaneCommand", stage=stage, planePath="/groundPlane", axis="Z",
    size=100.0, position=Gf.Vec3f(0, 0, 0.0), color=Gf.Vec3f(0.5),
)
light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
light.CreateIntensityAttr(1000)

# --- 2b) Cuarto 6x6 + cajas (misma geometria que tareas_room.sdf) ----------
# (centro, tamano) en metros; z=0.3 (altura 0.6), el lidar a ~0.2 m los corta.
ROOM = {
    "wall_n": ((-1.5, 2.25, 0.3), (6.0, 0.1, 0.6)),
    "wall_s": ((-1.5, -3.75, 0.3), (6.0, 0.1, 0.6)),
    "wall_e": ((1.5, -0.75, 0.3), (0.1, 6.0, 0.6)),
    "wall_w": ((-4.5, -0.75, 0.3), (0.1, 6.0, 0.6)),
    "box1": ((0.4, 1.1, 0.3), (0.6, 0.6, 0.6)),
    "box2": ((-3.5, 0.8, 0.3), (0.6, 0.6, 0.6)),
    "box3": ((-0.3, -3.0, 0.3), (0.9, 0.6, 0.6)),
    "box4": ((0.6, -2.0, 0.3), (0.5, 1.4, 0.6)),
}


def add_box(name, center, size):
    path = f"/Room/{name}"
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))
    cube = UsdGeom.Cube.Define(stage, path + "/geo")
    cube.GetSizeAttr().Set(1.0)                      # cubo unitario -> escalado da la caja
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.6, 0.6, 0.65)])


UsdGeom.Xform.Define(stage, "/Room")
for nm, (c, s) in ROOM.items():
    add_box(nm, c, s)
print(f"[F3] Cuarto con {len(ROOM)} cuerpos (4 paredes + 4 cajas)")

# --- 3) Grafo ROS2: clock + odom(GT) + TF odom->base + joint_states --------
chassis_prim = prim_path
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PublishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("PublishJoint", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishOdom.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishRawTF.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishJoint.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishRawTF.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishJoint.inputs:timeStamp"),
            ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
            ("ComputeOdom.outputs:position", "PublishRawTF.inputs:translation"),
            ("ComputeOdom.outputs:orientation", "PublishRawTF.inputs:rotation"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "clock"),
            ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(chassis_prim)]),
            ("PublishOdom.inputs:topicName", "odom"),
            ("PublishOdom.inputs:odomFrameId", "odom"),
            ("PublishOdom.inputs:chassisFrameId", "base_footprint"),
            ("PublishRawTF.inputs:topicName", "/tf"),
            ("PublishRawTF.inputs:parentFrameId", "odom"),
            ("PublishRawTF.inputs:childFrameId", "base_footprint"),
            ("PublishJoint.inputs:topicName", "joint_states"),
            ("PublishJoint.inputs:targetPrim", [usdrt.Sdf.Path(prim_path)]),
        ],
    },
)

# --- 3b) RTX lidar 2D montado en lidar_frame -> /scan ----------------------
# Buscar el prim lidar_frame del robot importado.
lidar_parent = None
for p in stage.Traverse():
    if p.GetName() == "lidar_frame":
        lidar_parent = p.GetPath().pathString
        break
if lidar_parent is None:
    lidar_parent = prim_path
    print("[F3] aviso: no se hallo lidar_frame; monto el lidar en el root")
print(f"[F3] lidar_frame: {lidar_parent}")

_, lidar = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="rtx_lidar",
    parent=lidar_parent,
    config=LIDAR_CONFIG,
    translation=(0.0, 0.0, 0.0),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),   # identidad rel. a lidar_frame (Z arriba)
)
hydra_texture = rep.create.render_product(lidar.GetPath(), [1, 1], name="JetautoLidar")
scan_writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
scan_writer.initialize(topicName="scan", frameId="lidar_frame")
scan_writer.attach([hydra_texture])
print(f"[F3] RTX lidar ({LIDAR_CONFIG}) publicando /scan en frame lidar_frame")

# --- 4) ROS2: suscriptor de /cmd_vel ---------------------------------------
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("isaac_jetauto_drive")
cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}


def _cmd_cb(msg: Twist):
    cmd["vx"] = msg.linear.x
    cmd["vy"] = msg.linear.y
    cmd["wz"] = msg.angular.z


node.create_subscription(Twist, "/cmd_vel", _cmd_cb, 10)

# --- 5) Arrancar fisica, articulacion, pose --------------------------------
timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

art = SingleArticulation(prim_path)
art.initialize()


def _quat_yaw(yaw):
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


px, py, pyaw, pz = args.x, args.y, args.yaw, args.z
try:
    art.set_world_pose(position=np.array([px, py, pz]), orientation=_quat_yaw(pyaw))
except Exception as e:
    print(f"[F3] aviso: no se pudo fijar pose inicial ({e})")

DT = 1.0 / 60.0
print("[F3] Listo. Publica /scan /odom /tf /joint_states /clock y escucha /cmd_vel.")
try:
    while simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        vx, vy, wz = cmd["vx"], cmd["vy"], cmd["wz"]
        c, s = math.cos(pyaw), math.sin(pyaw)
        px += (vx * c - vy * s) * DT
        py += (vx * s + vy * c) * DT
        pyaw += wz * DT
        art.set_world_pose(position=np.array([px, py, pz]), orientation=_quat_yaw(pyaw))
        art.set_linear_velocity(np.array([vx * c - vy * s, vx * s + vy * c, 0.0]))
        art.set_angular_velocity(np.array([0.0, 0.0, wz]))
        simulation_app.update()
except KeyboardInterrupt:
    pass
finally:
    rclpy.shutdown()
    timeline.stop()
    simulation_app.close()
