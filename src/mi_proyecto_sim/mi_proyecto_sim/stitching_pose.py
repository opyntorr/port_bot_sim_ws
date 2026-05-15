#!/usr/bin/env python3
"""
Stitcher por pose (proyeccion ortografica al plano del suelo).

Mejoras sobre la version original:
  - Feathering por distancia al centro (peso coseno): los bordes de cada foto
    contribuyen menos, reduciendo ghosting en zonas de overlap.
  - Multi-band blending (piramide Laplaciana): mezcla suave en multiples escalas
    de frecuencia para eliminar discontinuidades de color y detalle.
  - Correccion de offset por features (SIFT / ORB): en la zona de overlap con el
    canvas ya construido, busca correspondencias y calcula un offset de traslacion
    para corregir el error residual de pose del Optitrack.

JSON esperado por foto (producido por mision_dron.py):
{
  "x": 1.234, "y": -0.567, "z": 2.510, "yaw": 0.0,
  "stamp": "20260514_155200"
}

Salida: --output/panorama.png  y  --output/panorama_meta.yaml
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


def _load_camera_yaml(path: Path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    K = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(data['distortion_coefficients']['data'], dtype=np.float64).flatten()
    w = int(data['image_width'])
    h = int(data['image_height'])
    cam_rot = math.radians(float(data.get('camera_rotation_deg', 0.0)))
    return K, D, w, h, cam_rot


def _undistort(img, K, D):
    h, w = img.shape[:2]
    newK, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=0.0)
    und = cv2.undistort(img, K, D, None, newK)
    return und, newK


def _project_to_ground(img, newK, altitude_m, m_per_px):
    """Reproyecta imagen nadir al plano del suelo a resolucion m_per_px."""
    h_img, w_img = img.shape[:2]
    fx = newK[0, 0]
    fy = newK[1, 1]
    cx = newK[0, 2]
    cy = newK[1, 2]

    width_m  = altitude_m * w_img / fx
    height_m = altitude_m * h_img / fy

    w_out = max(1, int(round(width_m  / m_per_px)))
    h_out = max(1, int(round(height_m / m_per_px)))

    us, vs = np.meshgrid(np.arange(w_out, dtype=np.float32),
                         np.arange(h_out, dtype=np.float32))
    xg = (us - w_out / 2.0) * m_per_px
    yg = (vs - h_out / 2.0) * m_per_px
    map_x = (fx * (xg / altitude_m) + cx).astype(np.float32)
    map_y = (fy * (yg / altitude_m) + cy).astype(np.float32)
    ground = cv2.remap(
        img, map_x, map_y,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )
    return ground, width_m, height_m


def _rotate_image(img, yaw_rad):
    """Rota la imagen por yaw_rad expandiendo el canvas para no recortar."""
    h, w = img.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), math.degrees(yaw_rad), 1.0)
    cos = abs(M[0, 0])
    sin = abs(M[0, 1])
    nw = int((h * sin) + (w * cos))
    nh = int((h * cos) + (w * sin))
    M[0, 2] += (nw / 2.0) - w / 2.0
    M[1, 2] += (nh / 2.0) - h / 2.0
    rot = cv2.warpAffine(
        img, M, (nw, nh),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )
    return rot


def _distance_weight_mask(shape):
    """
    Mascara de peso por distancia al centro: centro=1, bordes=0 con caida coseno.
    Los pixeles negros (sin dato) tienen peso 0.
    """
    h, w = shape[:2]
    y = np.linspace(-1.0, 1.0, h, dtype=np.float32)
    x = np.linspace(-1.0, 1.0, w, dtype=np.float32)
    xx, yy = np.meshgrid(x, y)
    dist = np.clip(np.sqrt(xx ** 2 + yy ** 2), 0.0, 1.0)
    return np.cos(dist * (math.pi / 2.0))


def _build_detector():
    """Devuelve (detector, matcher, nombre). SIFT preferido, ORB como fallback."""
    for use_sift in (True, False):
        try:
            if use_sift:
                det = cv2.SIFT_create(nfeatures=3000)
                mat = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
                return det, mat, 'SIFT'
            else:
                det = cv2.ORB_create(nfeatures=3000)
                mat = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
                return det, mat, 'ORB'
        except Exception:
            continue
    return None, None, None


def _feature_offset(canvas_f32, weight_sum, patch, u0, v0, det, matcher):
    """
    Calcula (du, dv) en pixeles para corregir el offset de 'patch' respecto
    al canvas usando features en la zona de overlap.

    Devuelve (0, 0) si no hay suficiente overlap o pocos matches.
    Limita la correccion a ±MAX_OFFSET px para evitar correcciones erroneas.
    """
    MAX_OFFSET = 25
    MIN_OVERLAP_PX = 2000
    MIN_MATCHES = 10

    H, W = canvas_f32.shape[:2]
    h_p, w_p = patch.shape[:2]

    cu0, cv0_c = max(0, u0), max(0, v0)
    cu1, cv1_c = min(W, u0 + w_p), min(H, v0 + h_p)
    if cu0 >= cu1 or cv0_c >= cv1_c:
        return 0, 0

    overlap_w = weight_sum[cv0_c:cv1_c, cu0:cu1]
    if float(overlap_w.max()) < 0.05:
        return 0, 0

    sx0 = cu0 - u0
    sy0 = cv0_c - v0

    patch_crop = patch[sy0:sy0 + (cv1_c - cv0_c), sx0:sx0 + (cu1 - cu0)]
    ref_crop   = np.clip(canvas_f32[cv0_c:cv1_c, cu0:cu1], 0, 255).astype(np.uint8)

    mask_new = (np.any(patch_crop != 0, axis=2)).astype(np.uint8) * 255
    mask_ref = (overlap_w > 0.05).astype(np.uint8) * 255
    mask_both = cv2.bitwise_and(mask_new, mask_ref)

    if int(mask_both.sum()) // 255 < MIN_OVERLAP_PX:
        return 0, 0

    gray_ref = cv2.cvtColor(ref_crop,   cv2.COLOR_BGR2GRAY)
    gray_new = cv2.cvtColor(patch_crop, cv2.COLOR_BGR2GRAY)

    try:
        kp1, des1 = det.detectAndCompute(gray_ref, mask_both)
        kp2, des2 = det.detectAndCompute(gray_new, mask_both)

        if des1 is None or des2 is None or len(kp1) < MIN_MATCHES or len(kp2) < MIN_MATCHES:
            return 0, 0

        matches = matcher.knnMatch(des1, des2, k=2)
        good = [m for m, n in matches if m.distance < 0.75 * n.distance]
        if len(good) < MIN_MATCHES:
            return 0, 0

        pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in good])
        delta = pts1 - pts2

        # Mediana es robusta a outliers
        du = float(np.median(delta[:, 0]))
        dv = float(np.median(delta[:, 1]))

        if abs(du) > MAX_OFFSET or abs(dv) > MAX_OFFSET:
            return 0, 0

        return int(round(du)), int(round(dv))

    except Exception:
        return 0, 0


def _multiband_blend(canvas_f32, weight_sum, patch, weight_mask, u0, v0, num_levels=4):
    """
    Pega 'patch' en canvas_f32 en (u0, v0) usando multi-band blending
    (piramide Laplaciana). El alpha de mezcla en cada pixel viene de
    weight_mask normalizado por weight_sum acumulado.
    """
    H, W = canvas_f32.shape[:2]
    h_p, w_p = patch.shape[:2]

    cu0, cv0 = max(0, u0), max(0, v0)
    cu1, cv1 = min(W, u0 + w_p), min(H, v0 + h_p)
    if cu0 >= cu1 or cv0 >= cv1:
        return

    roi_w = cu1 - cu0
    roi_h = cv1 - cv0

    sx0 = cu0 - u0
    sy0 = cv0 - v0

    A = canvas_f32[cv0:cv1, cu0:cu1].copy()
    B = patch[sy0:sy0 + roi_h, sx0:sx0 + roi_w].astype(np.float32)

    w_old = weight_sum[cv0:cv1, cu0:cu1]          # (roi_h, roi_w)
    w_new = weight_mask[sy0:sy0 + roi_h, sx0:sx0 + roi_w]

    w_total = w_old + w_new
    alpha   = np.where(w_total > 1e-8, w_new / w_total, 0.0).astype(np.float32)
    alpha3  = np.stack([alpha] * 3, axis=-1)

    # Donde el canvas esta vacio, inicializamos con B para que la piramide sea coherente
    A_init = np.where(w_old[:, :, None] < 1e-6, B, A)

    # Limitar niveles segun tamano del ROI
    max_levels = max(1, int(math.log2(min(roi_h, roi_w) + 1)))
    levels = min(num_levels, max_levels)

    def gaussian_pyr(img, lvls):
        pyr = [img.copy()]
        cur = img.copy()
        for _ in range(lvls - 1):
            cur = cv2.pyrDown(cur)
            pyr.append(cur)
        return pyr

    def laplacian_pyr(img, lvls):
        g = gaussian_pyr(img, lvls)
        lp = []
        for i in range(lvls - 1):
            sz = (g[i].shape[1], g[i].shape[0])
            up = cv2.pyrUp(g[i + 1], dstsize=sz)
            lp.append(g[i] - up)
        lp.append(g[-1].copy())
        return lp

    lp_A = laplacian_pyr(A_init, levels)
    lp_B = laplacian_pyr(B,      levels)
    gp_a = gaussian_pyr(alpha3,  levels)

    blended = [la * (1.0 - ga) + lb * ga
               for la, lb, ga in zip(lp_A, lp_B, gp_a)]

    result = blended[-1]
    for i in range(levels - 2, -1, -1):
        sz = (blended[i].shape[1], blended[i].shape[0])
        result = cv2.pyrUp(result, dstsize=sz) + blended[i]

    canvas_f32[cv0:cv1, cu0:cu1] = np.clip(result, 0.0, 255.0)
    weight_sum[cv0:cv1, cu0:cu1] += w_new


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input',      required=True)
    ap.add_argument('--output',     required=True)
    ap.add_argument('--camera',     required=True)
    ap.add_argument('--resolution', type=float, default=0.005,
                    help='Resolucion del canvas (m/px). 0.005 = 5 mm/px.')
    ap.add_argument('--margin',     type=float, default=0.5,
                    help='Margen extra (m) a cada lado del canvas.')
    ap.add_argument('--levels',     type=int,   default=4,
                    help='Niveles de la piramide Laplaciana para multi-band blending.')
    ap.add_argument('--no-features', action='store_true',
                    help='Desactivar correccion de offset por features.')
    args = ap.parse_args()

    in_dir  = Path(args.input)
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    K, D, _, _, cam_rot = _load_camera_yaml(Path(args.camera))

    photos = sorted(in_dir.glob('*.png'))
    if not photos:
        print(f'[stitcher] no se encontraron .png en {in_dir}', file=sys.stderr)
        sys.exit(1)

    # ── 1. Proyectar + rotar + generar weight masks ──────────────────────────
    projected = []
    weight_masks = []
    poses = []

    for ph in photos:
        meta_path = ph.with_suffix('.json')
        if not meta_path.exists():
            print(f'[stitcher] omitiendo {ph.name}: falta {meta_path.name}', file=sys.stderr)
            continue
        with open(meta_path, 'r') as f:
            meta = json.load(f)
        img = cv2.imread(str(ph))
        if img is None:
            print(f'[stitcher] no se pudo leer {ph}', file=sys.stderr)
            continue
        und, newK = _undistort(img, K, D)
        altitude = float(meta['z'])
        if altitude <= 0.1:
            print(f'[stitcher] altura invalida ({altitude:.2f}m) en {ph.name}', file=sys.stderr)
            continue

        ground, _, _ = _project_to_ground(und, newK, altitude, args.resolution)
        if 'yaw' in meta:
            yaw = float(meta['yaw'])
        elif 'yaw_deg' in meta:
            yaw = math.radians(float(meta['yaw_deg']))
        else:
            yaw = 0.0
        rotated      = _rotate_image(ground, yaw + cam_rot)

        # Mascara de feathering: coseno de distancia al centro, 0 donde no hay dato
        wmask = _distance_weight_mask(rotated.shape)
        content = np.any(rotated != 0, axis=2).astype(np.float32)
        wmask  *= content

        projected.append(rotated)
        weight_masks.append(wmask)
        poses.append((float(meta['x']), float(meta['y']), yaw, rotated.shape))

    if not projected:
        print('[stitcher] sin fotos validas', file=sys.stderr)
        sys.exit(2)

    # ── 2. Canvas global ─────────────────────────────────────────────────────
    xs_min, xs_max, ys_min, ys_max = [], [], [], []
    for (cx_w, cy_w, _y, shp) in poses:
        h_p, w_p = shp[:2]
        hw = (w_p * args.resolution) / 2.0
        hh = (h_p * args.resolution) / 2.0
        xs_min.append(cx_w - hw);  xs_max.append(cx_w + hw)
        ys_min.append(cy_w - hh);  ys_max.append(cy_w + hh)

    x_min = min(xs_min) - args.margin
    x_max = max(xs_max) + args.margin
    y_min = min(ys_min) - args.margin
    y_max = max(ys_max) + args.margin

    W = int(math.ceil((x_max - x_min) / args.resolution))
    H = int(math.ceil((y_max - y_min) / args.resolution))

    canvas_f32  = np.zeros((H, W, 3), dtype=np.float32)
    weight_sum  = np.zeros((H, W),    dtype=np.float32)

    # ── 3. Feature detector ──────────────────────────────────────────────────
    det = matcher = None
    if not args.no_features:
        det, matcher, det_name = _build_detector()
        if det is not None:
            print(f'[stitcher] Feature detector: {det_name}')
        else:
            print('[stitcher] Sin detector de features disponible; saltando correccion.', file=sys.stderr)

    # ── 4. Pegar fotos: correccion features + multi-band blend ───────────────
    for img, wmask, (cx_w, cy_w, _y, _shp) in zip(projected, weight_masks, poses):
        h_p, w_p = img.shape[:2]

        # Centro de la foto en pixeles del canvas
        # canvas-X crece a la derecha = mundo-x; canvas-Y crece hacia abajo = -mundo-y
        cu     = int(round((cx_w - x_min) / args.resolution))
        cv_pix = int(round((y_max - cy_w) / args.resolution))
        u0 = cu     - w_p // 2
        v0 = cv_pix - h_p // 2

        # Correccion de offset por features (solo si ya hay contenido en el canvas)
        if det is not None and weight_sum.max() > 0.05:
            du, dv = _feature_offset(canvas_f32, weight_sum, img, u0, v0, det, matcher)
            if du != 0 or dv != 0:
                print(f'[stitcher] features offset ({du:+d}, {dv:+d}) px')
            u0 += du
            v0 += dv

        _multiband_blend(canvas_f32, weight_sum, img, wmask, u0, v0, num_levels=args.levels)

    # ── 5. Guardar resultado ─────────────────────────────────────────────────
    canvas = np.clip(canvas_f32, 0, 255).astype(np.uint8)
    panorama_path = out_dir / 'panorama.png'
    cv2.imwrite(str(panorama_path), canvas)

    meta_out = {
        'resolution':     float(args.resolution),
        'world_origin_x': float(x_min),
        'world_origin_y': float(y_min),
        'world_extent_x': float(x_max - x_min),
        'world_extent_y': float(y_max - y_min),
        'image_width':    int(W),
        'image_height':   int(H),
        'num_photos':     int(len(projected)),
    }
    with open(out_dir / 'panorama_meta.yaml', 'w') as f:
        yaml.dump(meta_out, f)

    print(f'[stitcher] OK -> {panorama_path}  ({W}x{H} px, {args.resolution} m/px, '
          f'extent={meta_out["world_extent_x"]:.2f}x{meta_out["world_extent_y"]:.2f} m)')


if __name__ == '__main__':
    main()
