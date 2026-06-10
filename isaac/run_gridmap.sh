#!/usr/bin/env bash
# Tarea 2 (GridMap) — lado cerebro. Requiere scene_gridmap.py ya corriendo.
# Levanta robot_state_publisher (arbol TF base->lidar), el nodo de mapeo y el KF
# como driver (mueve el robot en circulo mientras se mapea), igual que en Gazebo.
# Uso:  ./isaac/run_gridmap.sh
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$HERE/isaac_env.sh"

cleanup() { kill -INT "$GRID" 2>/dev/null; sleep 1; kill "$RSP" "$GRID" 2>/dev/null; }
trap cleanup EXIT

# arbol TF fijo base_footprint->base_link->lidar_link->lidar_frame
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(cat "$HERE/assets/jetauto.urdf")" >/tmp/isaac_rsp.log 2>&1 &
RSP=$!
# mapeo log-odds desde /scan
python3 "$HERE/gridmap_isaac.py" >/tmp/isaac_gridmap_node.log 2>&1 &
GRID=$!
sleep 2
echo ">> mapeando mientras el KF conduce el círculo (~90 s de sim)..."
python3 "$HERE/kf_control_isaac.py"        # foreground; al terminar, cleanup
echo ">> listo. Mapa en $HERE/figs/isaac_10_mapa_ocupacion.png"
