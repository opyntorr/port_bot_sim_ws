#!/usr/bin/env bash
# Diagnóstico de la cadena de pose/control de la misión real.
# Corre esto en otra terminal mientras la misión vuela:
#   ./diag_mision.sh [duracion_s]
#
# Registra, para cada eslabón de la cadena, frecuencia y último valor:
#   /optitrack/rigid_body  -> ¿OptiTrack vivo? ¿frame_id? ¿z sube al volar?
#   /odom                  -> ¿el hilo de odom del driver sigue vivo?
#   /odometry/filtered     -> ¿el fuser publica? ¿z > 0.4?
#   /drone1/target_position-> ¿la misión llegó a GOTO_WP?
#   /control               -> ¿el PID manda comandos != 0?

DURATION=${1:-120}
OUT_DIR="$(dirname "$0")/../mision_output/diag_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT_DIR"
echo "Registrando ${DURATION}s en $OUT_DIR"

TOPICS=(
  "/optitrack/rigid_body"
  "/odom"
  "/odometry/filtered"
  "/drone1/target_position"
  "/control"
)

PIDS=()
for t in "${TOPICS[@]}"; do
  name=$(echo "$t" | tr '/' '_' | sed 's/^_//')
  # echo con timestamp por mensaje (valores) — truncado para no llenar disco
  stdbuf -oL ros2 topic echo "$t" --no-arr 2>&1 \
    | stdbuf -oL awk '{ print strftime("%H:%M:%S"), $0 }' \
    > "$OUT_DIR/${name}_echo.log" &
  PIDS+=($!)
  # hz para detectar cuándo muere cada topic
  stdbuf -oL ros2 topic hz "$t" --window 20 2>&1 \
    | stdbuf -oL awk '{ print strftime("%H:%M:%S"), $0 }' \
    > "$OUT_DIR/${name}_hz.log" &
  PIDS+=($!)
done

sleep "$DURATION"
kill "${PIDS[@]}" 2>/dev/null
wait 2>/dev/null

echo
echo "=== Resumen ==="
for t in "${TOPICS[@]}"; do
  name=$(echo "$t" | tr '/' '_' | sed 's/^_//')
  hz_last=$(grep 'average rate' "$OUT_DIR/${name}_hz.log" | tail -1)
  hz_count=$(grep -c 'average rate' "$OUT_DIR/${name}_hz.log")
  echo "$t"
  echo "    último hz: ${hz_last:-SIN MENSAJES}"
  echo "    lecturas hz: $hz_count (si se detienen antes que las demás, ese topic murió)"
done
echo
echo "Logs completos en: $OUT_DIR"
echo "Claves a revisar:"
echo "  - odometry_filtered_echo.log: ¿position.z sube de 0.4 tras el despegue?"
echo "  - odom_hz.log vs optitrack_rigid_body_hz.log: ¿cuál muere primero?"
echo "  - control_echo.log: ¿twists distintos de cero cuando hay target?"
