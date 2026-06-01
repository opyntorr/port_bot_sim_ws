-- Cartographer 2D config para LOCALIZATION mode con prior congelado del dron.
-- Combina:
--   * frozen_trajectories: el .pbstream del dron queda como submap inmutable.
--   * pure_localization_trimmer: acota la memoria conservando solo los ultimos N
--     submaps nuevos del carrito.
-- El resultado: loop closure activo, drift corregido, memoria bounded.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Frames: usamos los nombres estandar del sistema.
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",

  -- TF: Cartographer publica map -> odom. La cadena odom -> base_footprint
  -- viene de la odometria del robot (publicada por otro nodo).
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,

  -- Sensores: solo LiDAR 2D filtrado.
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- Timeouts.
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,  -- 200 Hz
  trajectory_publish_period_sec = 30e-3,

  -- Multiplicadores para muestreo de sensores (1.0 = todos los mensajes).
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- Ajustar al rango efectivo de tu LiDAR (el filtro publica 190deg ~ 8m util).
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Submaps mas chicos para que el trimmer rote rapido y la memoria quede bajo control.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35

-- Voxel filter agresivo: menos puntos por scan = menos CPU y memoria.
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- Adaptive voxel filter (mantiene cierta densidad minima).
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.0

-- Scan matcher: pesos un poco mas altos a la rotacion para entornos simetricos.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- TRIMMER: clave para acotar memoria. Conserva solo los ultimos N submaps
-- NO congelados (los del dron sobreviven siempre).
-- IMPORTANTE: este key esta en TRAJECTORY_BUILDER (no en _2D), porque
-- pertenece al wrapper general TrajectoryBuilderOptions.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Pose graph: optimizacion global menos agresiva para no quemar CPU.
POSE_GRAPH.optimize_every_n_nodes = 50
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.global_sampling_ratio = 0.003

-- Trayectoria global del optimizador (deja correr loop closure entre nodos lejanos).
POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options
