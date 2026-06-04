"""Standalone yaw-convention validator + FOV calibration helper (Guide §8 + §5).

Usage:
    python3 -m mision.debug_yaw --input mision_output/16fotos --output mision_output/debug_yaw
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

from .io_utils import load_tiles, world_extent
from .place_pose import (
    PlaceConfig, CanvasSpec, footprint_pixels, make_canvas_spec,
    prepare_tile, assign_canvas_centers, paste_phase_a, annotate_indices,
)
from .preprocess import clahe_lab, edge_image, grid_mask
from .refine_window import mean_adjacency_score


def _build_canvas_for_variant(
    tiles, cfg: PlaceConfig
) -> Tuple[CanvasSpec, list]:
    extent = world_extent(tiles)
    fp_max = max(footprint_pixels(t.z, t.img_h, cfg.fov_v_deg, cfg.ppm) for t in tiles)
    spec = make_canvas_spec(extent, cfg, footprint_px_max=fp_max)
    placed = []
    for t in tiles:
        bgr = clahe_lab(t.img)
        p = prepare_tile(t, cfg, preproc_bgr=bgr)
        placed.append(p)
    assign_canvas_centers(tiles, spec, placed)
    return spec, placed


def _score_variant(spec, placed) -> float:
    """Mean NCC of edge-overlap regions across all adjacency pairs."""
    del spec
    return mean_adjacency_score(placed, edge_image)


def run_yaw_test(input_dir: Path, output_dir: Path, ppm: float, fov_v_deg: float) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    tiles = load_tiles(input_dir)
    print(f"[debug_yaw] Loaded {len(tiles)} tiles. "
          f"Yaw range: [{min(t.yaw_deg for t in tiles):+.2f}°, {max(t.yaw_deg for t in tiles):+.2f}°]")

    variants = [
        ("no_yaw", 0.0, 0.0),
        ("pos_yaw", +1.0, 0.0),
        ("neg_yaw", -1.0, 0.0),
        ("neg_yaw_plus_90", -1.0, 90.0),
        ("neg_yaw_minus_90", -1.0, -90.0),
        ("neg_yaw_180", -1.0, 180.0),
    ]

    scores = []
    for name, sign, off in variants:
        cfg = PlaceConfig(ppm=ppm, fov_v_deg=fov_v_deg, yaw_sign=sign, yaw_axis_offset_deg=off)
        # No-yaw variant overrides sign to 0 by setting yaw_sign=0:
        if name == "no_yaw":
            cfg.yaw_sign = 0.0
        spec, placed = _build_canvas_for_variant(tiles, cfg)
        canvas, _ = paste_phase_a(spec, placed)
        canvas = annotate_indices(canvas, placed)
        cv2.imwrite(str(output_dir / f"debug_yaw_{name}.png"), canvas)
        score = _score_variant(spec, placed)
        scores.append((name, score))
        print(f"  variant={name:<22}  mean_NCC={score:+.4f}")

    print()
    scores.sort(key=lambda x: -x[1])
    print(f"[debug_yaw] BEST variant: {scores[0][0]}  (NCC={scores[0][1]:+.4f})")
    print(f"[debug_yaw] Visually compare debug_yaw_*.png in {output_dir}")


def _estimate_fov_from_grid(tile_img: np.ndarray, tile_z: float, grid_spacing_m: float) -> float | None:
    """Detect grid lines via orange mask, return calibrated fov_v_deg, or None on failure."""
    gm = grid_mask(tile_img)
    # Hough lines.
    lines = cv2.HoughLinesP(gm, 1, np.pi / 360.0, threshold=80, minLineLength=80, maxLineGap=20)
    if lines is None or len(lines) < 4:
        return None
    angles = np.array([math.atan2(y2 - y1, x2 - x1) for x1, y1, x2, y2 in lines[:, 0]])
    # Cluster angles into two families (perpendicular).
    angles_mod = np.mod(angles, np.pi)
    fam_a = angles_mod < (np.pi / 4)
    fam_b = ~fam_a
    # If clearly horizontal/vertical, measure spacing along the *normal* of each family.
    H, W = gm.shape
    def cluster_offsets(family_lines):
        # Project each line's midpoint onto its family normal direction.
        if len(family_lines) == 0:
            return None
        avg_ang = np.mean([math.atan2(y2 - y1, x2 - x1) for x1, y1, x2, y2 in family_lines[:, 0]])
        nx, ny = -math.sin(avg_ang), math.cos(avg_ang)
        offsets = []
        for x1, y1, x2, y2 in family_lines[:, 0]:
            mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            offsets.append(mx * nx + my * ny)
        offsets = np.array(sorted(offsets))
        # Merge near-duplicates (<10 px).
        merged = []
        for o in offsets:
            if not merged or abs(o - merged[-1]) > 10:
                merged.append(o)
        if len(merged) < 2:
            return None
        diffs = np.diff(merged)
        return float(np.median(diffs))

    fa = lines[fam_a]; fb = lines[fam_b]
    spacing_a = cluster_offsets(fa)
    spacing_b = cluster_offsets(fb)
    candidates = [s for s in (spacing_a, spacing_b) if s is not None]
    if not candidates:
        return None
    pixel_spacing_px = float(np.median(candidates))
    # fov_v = 2 * atan( (H/2) * grid_spacing_m / (z * pixel_spacing_px) )
    fov_v_rad = 2.0 * math.atan((H / 2.0) * grid_spacing_m / (tile_z * pixel_spacing_px))
    return math.degrees(fov_v_rad)


def calibrate_fov(input_dir: Path, grid_spacing_m: float = 0.5, candidates: List[int] = (5, 6, 9, 10)) -> float | None:
    tiles = load_tiles(input_dir)
    by_idx = {t.idx: t for t in tiles}
    estimates = []
    for idx in candidates:
        if idx not in by_idx:
            continue
        t = by_idx[idx]
        fov = _estimate_fov_from_grid(t.img, t.z, grid_spacing_m)
        if fov is not None and 30.0 < fov < 110.0:
            estimates.append((idx, fov))
            print(f"  wp_{idx:02d}: fov_v ≈ {fov:.2f}°")
    if not estimates:
        print("[debug_yaw] FOV calibration failed (grid detection unreliable on the tried tiles)")
        return None
    median_fov = float(np.median([f for _, f in estimates]))
    print(f"[debug_yaw] Calibrated fov_v ≈ {median_fov:.2f}°  (from {len(estimates)} tiles)")
    return median_fov


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, type=Path)
    ap.add_argument("--output", required=True, type=Path)
    ap.add_argument("--ppm", type=float, default=500.0)
    ap.add_argument("--fov-v-deg", type=float, default=None,
                    help="Skip auto-calibration if provided.")
    ap.add_argument("--grid-spacing-m", type=float, default=0.5)
    args = ap.parse_args()

    fov = args.fov_v_deg
    if fov is None:
        fov = calibrate_fov(args.input, grid_spacing_m=args.grid_spacing_m)
        if fov is None:
            print("[debug_yaw] Falling back to fov_v_deg=66.0")
            fov = 66.0
    else:
        print(f"[debug_yaw] Using provided fov_v_deg={fov:.2f}")

    run_yaw_test(args.input, args.output, args.ppm, fov)


if __name__ == "__main__":
    main()
