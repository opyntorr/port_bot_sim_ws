# CLAUDE.md — Reglas para este repositorio

## Commits
- No incluyas ninguna referencia a Claude en commits: ni `Co-Authored-By`, ni menciones en el mensaje.
- Mensajes de commit en español, estilo convencional (`feat:`, `fix:`, `docs:`, etc.).

## Código
- No añadas comentarios que expliquen qué hace el código; solo añade comentarios cuando el *por qué* sea no obvio.
- No añadas manejo de errores para escenarios que no pueden ocurrir.
- No crees archivos de documentación (README, notas, análisis) a menos que se pida explícitamente.

## Archivos y estructura
- Los archivos de salida de misiones (`mision_output/`) son datos generados, nunca van al repo.
- Los archivos `__pycache__/` y `*.pyc` nunca van al repo.
- Todo el código fuente relevante vive en `src/`.

## Entorno
- El workspace de desarrollo es `/home/opyntorr/agv_uav_project/`.
- Dentro del contenedor Docker el workspace es `/ros2_ws/`, donde `src/` es el mismo directorio montado.
- Para probar cambios en Docker: `colcon build --packages-select <paquete> && source install/setup.bash`.

## Paquetes principales
- `src/mi_proyecto_sim/` — simulación Gazebo: mundo, modelos, misión del dron, stitching.
- `src/tello_control_pos/` — controlador de posición, OptiTrack simulado, pose fuser.
- `src/demo_tello_sim/` — drivers y calibración del Tello real.

## Stitching
- El stitcher activo es `stitching_pose.py` (proyección ortográfica por pose + multi-band blending).
- Se invoca con `--camera config/camera_tello_sim.yaml --resolution 0.005` en simulación.
- El stitcher de homografías en cadena (`stitch_images.py`) NO sirve para cuadrículas — solo para vuelos lineales.
