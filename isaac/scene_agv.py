#!/usr/bin/env python3
# F1 — Escena AGV (JetAuto mecanum) en Isaac Sim 4.5.
#
# Importa el URDF (isaac/assets/jetauto.urdf), arma el grafo ROS2 que publica
#   /clock, /odom (ground-truth), TF odom->base_footprint y /joint_states,
# y conduce la base de forma HOLONOMICA CINEMATICA desde /cmd_vel (override de la
# velocidad del root de la articulacion: vx, vy, wz). No se depende de los 48
# rodillos para el desplazamiento lateral (ver plan, decision #2).
#
# Correr:   source isaac/isaac_env.sh
#           $ISAACSIM/python.sh isaac/scene_agv.py
#           $ISAACSIM/python.sh isaac/scene_agv.py --headless
# Cerebro/RViz/teleop (otra terminal, isaac_env.sh sourceado):
#           ros2 launch (ver isaac/jetauto_isaac_view.launch.py)
import argparse
import math
import os

from isaacsim import SimulationApp

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "assets", "jetauto.urdf")

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--urdf", default=URDF_PATH)
parser.add_argument("--x", type=float, default=0.0)
parser.add_argument("--y", type=float, default=0.0)
parser.add_argument("--z", type=float, default=0.06)          # un poco arriba para asentarse
parser.add_argument("--yaw", type=float, default=0.0)         # rad
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": args.headless})

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.timeline
import usdrt.Sdf
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, PhysxSchema, Sdf, UsdLux, UsdPhysics

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
simulation_app.update()

# --- 1) Importar el URDF ---------------------------------------------------
_, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = False                # base flotante: la conducimos por velocidad
import_config.distance_scale = 1.0
_, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=args.urdf,
    import_config=import_config,
    get_articulation_root=True,
)
print(f"[F1] URDF importado en prim: {prim_path}")   # esperado: /jetauto

stage = omni.usd.get_context().get_stage()

# --- 1b) Materiales: verde -> aluminio anodizado, negro -> plastico mate -------
# (el importador de URDF deja el robot blanco; ver jetauto_materials.py)
from jetauto_materials import apply_jetauto_materials
apply_jetauto_materials(stage, prim_path)

# --- 2) Escena de fisica + piso + luz (F1 = mundo vacio; el laberinto es F3) ---
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(0.0)   # base cinematica: la pose la fijamos nosotros
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

# --- 3) Grafo ROS2: clock + odom(ground-truth) + TF odom->base + joint_states ---
# El importador con get_articulation_root=True ya devuelve el link raiz
# (p.ej. /jetauto/base_footprint), que es a la vez la articulacion y el chasis.
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

# --- 4) ROS2: suscriptor de /cmd_vel (rclpy compilado por Omniverse) ---------
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

# --- 5) Arrancar fisica (timeline, patron de urdf_import.py), articulacion, pose ----
# NOTA: usamos el timeline directo (NO SimulationContext.initialize_physics, que
# segfaulteaba al crear esta articulacion).
timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()   # un paso para que la fisica/articulacion sea valida

art = SingleArticulation(prim_path)
art.initialize()
def _quat_yaw(yaw):  # wxyz
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


# Estado integrado de la base (holonomica cinematica); z fijo al de reposo.
px, py, pyaw, pz = args.x, args.y, args.yaw, args.z
try:
    art.set_world_pose(position=np.array([px, py, pz]), orientation=_quat_yaw(pyaw))
except Exception as e:
    print(f"[F1] aviso: no se pudo fijar pose inicial ({e})")

DT = 1.0 / 60.0
print("[F1] Listo. Publica /odom /tf /joint_states /clock y escucha /cmd_vel.")
try:
    while simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        vx, vy, wz = cmd["vx"], cmd["vy"], cmd["wz"]
        # integrar la pose en el marco mundo y teleportar (no pelea con la fisica)
        c, s = math.cos(pyaw), math.sin(pyaw)
        px += (vx * c - vy * s) * DT
        py += (vx * s + vy * c) * DT
        pyaw += wz * DT
        art.set_world_pose(position=np.array([px, py, pz]), orientation=_quat_yaw(pyaw))
        # velocidades -> para que /odom.twist refleje el comando
        art.set_linear_velocity(np.array([vx * c - vy * s, vx * s + vy * c, 0.0]))
        art.set_angular_velocity(np.array([0.0, 0.0, wz]))
        simulation_app.update()
except KeyboardInterrupt:
    pass
finally:
    rclpy.shutdown()
    timeline.stop()
    simulation_app.close()
