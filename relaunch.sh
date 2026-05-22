#!/usr/bin/env bash
# Mata cualquier estado residual de Gazebo/ROS antes de relanzar.
# Uso:  ./relaunch.sh                         # solo carrito (default)
#       ./relaunch.sh simulacion              # launch completo con dron
#       ./relaunch.sh control_con_obstaculos  # spawn de obstaculos + control

set -e

LAUNCH_NAME="${1:-solo_carrito}.launch.py"

echo "[relaunch] Matando procesos residuales..."
pkill -9 -f "ign gazebo"        2>/dev/null || true
pkill -9 -f "ruby.*ign"         2>/dev/null || true
pkill -9 -f "gz-sim"            2>/dev/null || true
pkill -9 -f "parameter_bridge"  2>/dev/null || true
pkill -9 -f "ros_gz_sim"        2>/dev/null || true
pkill -9 -f "amcl_localizer"    2>/dev/null || true
pkill -9 -f "slam_occupancy"    2>/dev/null || true
pkill -9 -f "rviz2"             2>/dev/null || true

sleep 1

# Reafirmar permisos de ejecucion en scripts Python del paquete
# (algunos editores quitan el bit +x al guardar).
SCRIPTS_DIR="$(dirname "$0")/src/mi_proyecto_sim/mi_proyecto_sim"
if [ -d "$SCRIPTS_DIR" ]; then
    chmod +x "$SCRIPTS_DIR"/*.py 2>/dev/null || true
fi

REMAINING=$(pgrep -fa "ign|gz-sim|ros_gz" || true)
if [ -n "$REMAINING" ]; then
    echo "[relaunch] WARNING: procesos aun vivos:"
    echo "$REMAINING"
fi

echo "[relaunch] Lanzando $LAUNCH_NAME ..."
ros2 launch mi_proyecto_sim "$LAUNCH_NAME"
