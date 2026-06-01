#!/usr/bin/env python3
"""
Convierte un mapa PGM + YAML (formato ROS map_server) a un .pbstream de
Cartographer 2D con un unico submap.

Uso:
  1. Una sola vez:
       bash tools/setup_carto_protos.sh
       # Tambien: apt install -y python3-protobuf  (o pip3 install protobuf)
  2. Luego:
       python3 tools/pgm_to_pbstream.py \\
           --pgm  maps/mapa_mision.pgm \\
           --yaml maps/mapa_mision.yaml \\
           --out  maps/mapa_mision.pbstream

El .pbstream se carga con `-load_state_filename ... -load_frozen_state true`.
El flag load_frozen_state al INICIO marca como congeladas todas las
trayectorias del prior (no necesitamos codificarlo en el .pbstream).
"""
import argparse
import gzip
import struct
import sys
from pathlib import Path

import numpy as np
import yaml

# Importar los _pb2.py generados por setup_carto_protos.sh
TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR / 'carto_protos'))

try:
    # Estructura plana en 2.0.0: todo bajo cartographer/mapping/proto/.
    from cartographer.mapping.proto import serialization_pb2
    from cartographer.mapping.proto import pose_graph_pb2
    from cartographer.mapping.proto import submap_pb2          # contiene Submap, Submap2D
    from cartographer.mapping.proto import grid_2d_pb2          # contiene Grid2D
    from cartographer.transform.proto import transform_pb2  # noqa: F401 (transitivo)
except ImportError as e:
    sys.stderr.write(
        f'ERROR: protos de Cartographer no encontrados ({e}).\n'
        f'Pasos:\n'
        f'  1. bash tools/setup_carto_protos.sh\n'
        f'  2. apt install -y python3-protobuf   (o pip3 install protobuf)\n'
    )
    sys.exit(1)

import cv2

# Constantes del formato .pbstream
MAGIC_NUMBER = 0x7B1D1F7B5BF501DB
FORMAT_VERSION = 1

# Constantes de Cartographer probability grid
# Cell value = 0      -> unknown
# Cell value = 1      -> correspondence_cost minimo (mas ocupado)
# Cell value = 32767  -> correspondence_cost maximo (mas libre)
VALUE_OCCUPIED = 1
VALUE_FREE = 32767
VALUE_UNKNOWN = 0


def pgm_to_grid_cells(img: np.ndarray, occupied_thresh: float, free_thresh: float,
                      negate: int) -> np.ndarray:
    """Convierte la imagen PGM a int32 con los valores de celda esperados
    por Grid2D.cells (repeated int32 con valores in [0, 32767])."""
    p = img.astype(np.float32) / 255.0
    if negate == 0:
        p_occ = 1.0 - p
    else:
        p_occ = p

    cells = np.full(img.shape, VALUE_UNKNOWN, dtype=np.int32)
    cells[p_occ > occupied_thresh] = VALUE_OCCUPIED
    cells[p_occ < free_thresh] = VALUE_FREE
    return cells


def build_grid2d(cells: np.ndarray, resolution: float,
                 origin_x: float, origin_y: float):
    """Construye un mensaje Grid2D que envuelve la rejilla."""
    height, width = cells.shape

    # Cartographer indexa con (0,0) en la esquina top-left de la imagen y
    # `max` es la posicion en mundo de esa esquina (mayor x, mayor y...
    # ojo, en Cartographer la convencion es: max es donde col=0 y row=0).
    # Para nuestra PGM, top-left tiene x=origin.x e y=origin.y+H*res.
    max_x = origin_x + width * resolution
    max_y = origin_y + height * resolution

    grid = grid_2d_pb2.Grid2D()
    grid.limits.resolution = resolution
    grid.limits.max.x = max_x
    grid.limits.max.y = max_y
    grid.limits.cell_limits.num_x_cells = width
    grid.limits.cell_limits.num_y_cells = height

    # Flatten en orden row-major (top-to-bottom, left-to-right).
    grid.cells.extend(cells.flatten().tolist())

    # known_cells_box: bounding box de celdas conocidas (no unknown).
    known_mask = cells != VALUE_UNKNOWN
    if known_mask.any():
        rows = np.where(known_mask.any(axis=1))[0]
        cols = np.where(known_mask.any(axis=0))[0]
        grid.known_cells_box.min_x = int(cols.min())
        grid.known_cells_box.max_x = int(cols.max())
        grid.known_cells_box.min_y = int(rows.min())
        grid.known_cells_box.max_y = int(rows.max())
    else:
        grid.known_cells_box.min_x = 0
        grid.known_cells_box.max_x = width - 1
        grid.known_cells_box.min_y = 0
        grid.known_cells_box.max_y = height - 1

    grid.min_correspondence_cost = 0.1
    grid.max_correspondence_cost = 0.9
    # Discriminar el oneof: usamos ProbabilityGrid (no TSDF). El mensaje
    # ProbabilityGrid esta vacio; solo lo "tocamos" para activar el oneof.
    grid.probability_grid_2d.SetInParent()

    return grid


def build_submap2d(grid):
    """Empaqueta el Grid2D en un Submap2D."""
    sub2d = submap_pb2.Submap2D()
    sub2d.local_pose.translation.x = 0.0
    sub2d.local_pose.translation.y = 0.0
    sub2d.local_pose.translation.z = 0.0
    sub2d.local_pose.rotation.w = 1.0
    sub2d.num_range_data = 1  # >=1 para que Cartographer lo considere valido
    sub2d.finished = True
    sub2d.grid.CopyFrom(grid)
    return sub2d


def build_pose_graph(global_pose=(0.0, 0.0)):
    """PoseGraph minimo con UNA trayectoria conteniendo UN submap."""
    pg = pose_graph_pb2.PoseGraph()
    traj = pg.trajectory.add()
    traj.trajectory_id = 0

    sm = traj.submap.add()
    sm.submap_index = 0
    sm.pose.translation.x = float(global_pose[0])
    sm.pose.translation.y = float(global_pose[1])
    sm.pose.translation.z = 0.0
    sm.pose.rotation.w = 1.0
    # x, y, z = 0 por default

    return pg


def build_all_trajectory_builder_options():
    """Una entrada placeholder por trayectoria (requerido por el deserializador)."""
    from cartographer.mapping.proto import trajectory_builder_options_pb2
    opts = trajectory_builder_options_pb2.AllTrajectoryBuilderOptions()
    entry = opts.options_with_sensor_ids.add()
    entry.trajectory_builder_options.SetInParent()
    return opts


def write_message(f, msg):
    """[uint64 LE: tamano gzip] [bytes: gzip(serialized proto)]"""
    raw = msg.SerializeToString()
    compressed = gzip.compress(raw)
    f.write(struct.pack('<Q', len(compressed)))
    f.write(compressed)


def write_pbstream(out_path: Path, pose_graph, all_traj_opts, submap_2d):
    with out_path.open('wb') as f:
        # 1. Magic number (8 bytes, little-endian uint64)
        f.write(struct.pack('<Q', MAGIC_NUMBER))

        # 2. SerializationHeader (compressed proto)
        header = serialization_pb2.SerializationHeader()
        header.format_version = FORMAT_VERSION
        write_message(f, header)

        # 3. PoseGraph
        sd = serialization_pb2.SerializedData()
        sd.pose_graph.CopyFrom(pose_graph)
        write_message(f, sd)

        # 4. AllTrajectoryBuilderOptions
        sd = serialization_pb2.SerializedData()
        sd.all_trajectory_builder_options.CopyFrom(all_traj_opts)
        write_message(f, sd)

        # 5. El submap propiamente dicho
        sd = serialization_pb2.SerializedData()
        sd.submap.submap_id.trajectory_id = 0
        sd.submap.submap_id.submap_index = 0
        sd.submap.submap_2d.CopyFrom(submap_2d)
        write_message(f, sd)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pgm', required=True, type=Path)
    ap.add_argument('--yaml', required=True, type=Path)
    ap.add_argument('--out', required=True, type=Path)
    ap.add_argument('--downsample', type=float, default=0.05,
                    help='Resolucion destino (m/celda). Default 0.05.')
    args = ap.parse_args()

    with args.yaml.open('r') as f:
        meta = yaml.safe_load(f)
    src_res = float(meta['resolution'])
    origin = meta['origin']
    origin_x, origin_y = float(origin[0]), float(origin[1])
    occupied_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.196))
    negate = int(meta.get('negate', 0))

    print(f'PGM: {args.pgm}')
    print(f'  resolucion nativa: {src_res} m/px')
    print(f'  origen: ({origin_x:.3f}, {origin_y:.3f})')
    print(f'  thresholds: occ={occupied_thresh}, free={free_thresh}, negate={negate}')

    img = cv2.imread(str(args.pgm), cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f'ERROR: no se pudo leer {args.pgm}')
    print(f'  tamano original: {img.shape[1]}x{img.shape[0]} px')

    if args.downsample > src_res:
        scale = src_res / args.downsample
        new_w = max(1, int(img.shape[1] * scale))
        new_h = max(1, int(img.shape[0] * scale))
        img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        out_res = args.downsample
        print(f'  downsampleado a: {new_w}x{new_h} px @ {out_res} m/px')
    else:
        out_res = src_res

    cells = pgm_to_grid_cells(img, occupied_thresh, free_thresh, negate)
    n_known = int((cells != VALUE_UNKNOWN).sum())
    print(f'  celdas conocidas (occ+free): {n_known} ({100*n_known/cells.size:.1f}%)')

    grid = build_grid2d(cells, out_res, origin_x, origin_y)
    submap_2d = build_submap2d(grid)
    pose_graph = build_pose_graph(global_pose=(0.0, 0.0))
    all_traj_opts = build_all_trajectory_builder_options()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_pbstream(args.out, pose_graph, all_traj_opts, submap_2d)
    size_mb = args.out.stat().st_size / (1024 * 1024)
    print(f'OK -> {args.out} ({size_mb:.2f} MB)')


if __name__ == '__main__':
    main()
