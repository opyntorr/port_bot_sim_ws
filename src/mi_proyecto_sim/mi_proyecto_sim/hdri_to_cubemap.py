#!/usr/bin/env python3
"""
Convierte un HDRI equirectangular (.hdr) en 6 caras de cubemap PNG
listas para usar como skybox en Ignition Gazebo.

Uso:
    python3 hdri_to_cubemap.py INPUT.hdr OUTPUT_DIR [--size 1024]
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


FACES = {
    'px': lambda u, v: np.stack([np.ones_like(u),       -v,                -u], axis=-1),
    'nx': lambda u, v: np.stack([-np.ones_like(u),      -v,                 u], axis=-1),
    'py': lambda u, v: np.stack([u,                      np.ones_like(u),   v], axis=-1),
    'ny': lambda u, v: np.stack([u,                     -np.ones_like(u),  -v], axis=-1),
    'pz': lambda u, v: np.stack([u,                     -v,   np.ones_like(u)], axis=-1),
    'nz': lambda u, v: np.stack([-u,                    -v,  -np.ones_like(u)], axis=-1),
}


def sample_equirect(equirect: np.ndarray, dirs: np.ndarray) -> np.ndarray:
    x, y, z = dirs[..., 0], dirs[..., 1], dirs[..., 2]
    r = np.sqrt(x * x + y * y + z * z)
    theta = np.arctan2(x, z)          # azimut, [-pi, pi]
    phi = np.arcsin(np.clip(y / r, -1, 1))  # elevación, [-pi/2, pi/2]

    H, W = equirect.shape[:2]
    u = (theta / (2 * np.pi) + 0.5) * W
    v = (0.5 - phi / np.pi) * H

    map_x = u.astype(np.float32)
    map_y = v.astype(np.float32)
    return cv2.remap(equirect, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)


def tonemap(hdr: np.ndarray, exposure: float, gamma: float) -> np.ndarray:
    img = hdr * (2.0 ** exposure)
    img = img / (1.0 + img)                  # Reinhard
    img = np.clip(img, 0.0, 1.0) ** (1.0 / gamma)
    return (img * 255.0).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input', type=Path)
    ap.add_argument('output_dir', type=Path)
    ap.add_argument('--size', type=int, default=1024, help='Tamaño en px de cada cara')
    ap.add_argument('--exposure', type=float, default=0.0, help='Stops de exposición (+/-)')
    ap.add_argument('--gamma', type=float, default=2.2)
    args = ap.parse_args()

    if not args.input.exists():
        sys.exit(f'No existe: {args.input}')

    equirect = cv2.imread(str(args.input), cv2.IMREAD_ANYDEPTH | cv2.IMREAD_COLOR)
    if equirect is None:
        sys.exit(f'OpenCV no pudo leer {args.input} (¿es .hdr/.exr válido?)')
    if equirect.dtype != np.float32:
        equirect = equirect.astype(np.float32) / 255.0
    print(f'Cargado {args.input.name}: {equirect.shape}, dtype={equirect.dtype}')

    args.output_dir.mkdir(parents=True, exist_ok=True)

    lin = np.linspace(-1.0, 1.0, args.size, dtype=np.float32)
    u_grid, v_grid = np.meshgrid(lin, lin)

    for name, dir_func in FACES.items():
        dirs = dir_func(u_grid, v_grid)
        face_hdr = sample_equirect(equirect, dirs)
        face = tonemap(face_hdr, args.exposure, args.gamma)
        out_path = args.output_dir / f'{name}.png'
        cv2.imwrite(str(out_path), face)
        print(f'  → {out_path.name}')

    print(f'Listo. 6 caras en {args.output_dir}')


if __name__ == '__main__':
    main()
