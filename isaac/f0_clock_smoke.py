#!/usr/bin/env python3
# F0 — Smoke test del bridge ROS2 de Isaac Sim 4.5.
# Publica /clock mientras la sim está en Play, para confirmar que el bridge ROS2
# inicializa y que el dominio (ROS_DOMAIN_ID, heredado del entorno) es el correcto.
#
# Correr:   source isaac/isaac_env.sh
#           $ISAACSIM/python.sh isaac/f0_clock_smoke.py            # con ventana
#           $ISAACSIM/python.sh isaac/f0_clock_smoke.py --headless # sin ventana
# Verificar (otra terminal, con isaac_env.sh sourceado):
#           ros2 topic echo /clock        # debe avanzar
#           ros2 topic hz   /clock
import argparse

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="sin ventana (GPU igual requerida)")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": args.headless})

import omni.graph.core as og
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# Grafo mínimo: cada tick de playback -> publica el reloj de simulación en /clock.
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "clock"),
        ],
    },
)

simulation_app.update()
sim = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
sim.initialize_physics()
sim.play()

print("[F0] Publicando /clock (Ctrl+C para salir). En otra terminal: ros2 topic echo /clock")
try:
    while simulation_app.is_running():
        sim.step(render=not args.headless)
except KeyboardInterrupt:
    pass
finally:
    sim.stop()
    simulation_app.close()
