#!/usr/bin/env bash
# Arranque del stack desde la Jetson.
#
# Requisitos previos:
#   - Raspberry encendida y conectada por Ethernet (10.0.0.2)
#   - Raspberry con robot_bringup.sh corriendo (o instalado el servicio systemd)
#
# Uso:
#   ./jetson_bringup.sh                # solo lanza el network_bridge
#   ./jetson_bringup.sh mision_real    # bridge + lanza mision_real.launch.py
#   ./jetson_bringup.sh stop           # detiene el network_bridge

set -e

ROBOT_IP="10.0.0.2"
ROS_DOMAIN="30"
WS="/home/jetson/ros2_ws"
PROJECT_WS="/home/jetson/agv_uav_project"
BRIDGE_CONFIG="$WS/src/network_bridge/config/Udp2.yaml"
BRIDGE_LOG="/tmp/network_bridge.log"

source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
[ -f "$PROJECT_WS/install/setup.bash" ] && source "$PROJECT_WS/install/setup.bash"
export ROS_DOMAIN_ID="$ROS_DOMAIN"

ACTION="${1:-start}"

stop_bridge() {
    echo "[jetson_bringup] Deteniendo network_bridge..."
    pkill -f "network_bridge --ros-args" 2>/dev/null || true
    sleep 1
    echo "[jetson_bringup] Detenido."
}

if [ "$ACTION" = "stop" ]; then
    stop_bridge
    exit 0
fi

echo "[jetson_bringup] Verificando conexion con la Raspberry ($ROBOT_IP)..."
if ! ping -c 2 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "ERROR: Raspberry no responde en $ROBOT_IP"
    echo "  - Revisa que este encendida"
    echo "  - Revisa el cable Ethernet"
    exit 1
fi
echo "[jetson_bringup] OK - Raspberry en linea."

stop_bridge

echo "[jetson_bringup] Lanzando network_bridge en background..."
nohup ros2 run network_bridge network_bridge \
    --ros-args --params-file "$BRIDGE_CONFIG" \
    > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!
echo "[jetson_bringup] network_bridge PID: $BRIDGE_PID  (log: $BRIDGE_LOG)"

echo "[jetson_bringup] Esperando topics del robot (max 20s)..."
FOUND=0
for i in $(seq 1 20); do
    if ros2 topic list 2>/dev/null | grep -q "^/odom$"; then
        FOUND=1
        break
    fi
    sleep 1
done

if [ "$FOUND" -eq 1 ]; then
    echo "[jetson_bringup] Topics activos:"
    ros2 topic list | sed 's/^/    /'
else
    echo "[jetson_bringup] AVISO: aun no se ven topics del robot."
    echo "  - Revisa que robot_bringup.sh este corriendo en la Raspberry"
    echo "  - Mira el log: tail -f $BRIDGE_LOG"
fi

if [ "$ACTION" != "start" ]; then
    LAUNCH_NAME="${ACTION}.launch.py"
    echo ""
    echo "[jetson_bringup] Lanzando $LAUNCH_NAME ..."
    ros2 launch mi_proyecto_sim "$LAUNCH_NAME"
fi
