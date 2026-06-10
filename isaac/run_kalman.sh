#!/usr/bin/env bash
# Tarea 1 (Kalman) — lado cerebro. Requiere scene_agv.py ya corriendo.
# Uso:  ./isaac/run_kalman.sh
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$HERE/isaac_env.sh"
echo ">> KF conduciendo el círculo (~90 s de sim); guarda isaac_01..03 en figs/"
python3 "$HERE/kf_control_isaac.py"
echo ">> listo. Figuras en $HERE/figs/"
