"""Debug script: visualise tape-snap phase detection vs global reference grid.

For each tile, produces an image with three panels:
  Left   — warped BGR with cyan lines at detected tape peaks and orange lines
            at expected positions from the global reference grid.
  Middle — de-rotated tape mask (what the auto method projects).
  Right  — Y-projection with cyan peaks detected, orange = global grid expected,
            and a green dashed line at the tile's canvas phase.

Run inside the Docker container:
    python3 -m mision.debug_snap_grid --input mision_output/55fotos \
        --output mision_output/debug_grid
"""
from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from .io_utils import load_tiles, world_extent
from .place_pose import (
    PlaceConfig, PlacedTile, footprint_pixels, make_canvas_spec,
    prepare_tile, assign_canvas_centers,
)
from .preprocess import clahe_lab, estimate_tile_yaw
from .tape_snap import (
    _tape_mask, _tape_phase_derotate, _tape_phase_hough,
    _find_tape_peaks, _phase_from_peaks, _robust_ref_phase,
    _estimate_period,
)

_CYAN   = (255, 255,   0)
_ORANGE = (  0, 128, 255)
_GREEN  = (  0, 220,   0)
_WHITE  = (255, 255, 255)
_RED    = (  0,   0, 255)


def _draw_hlines(img: np.ndarray, ys: List[float], color, thickness: int = 1) -> None:
    h, w = img.shape[:2]
    for y in ys:
        yi = int(round(y))
        if 0 <= yi < h:
            cv2.line(img, (0, yi), (w - 1, yi), color, thickness)


def _projection_image(proj: np.ndarray, h: int) -> np.ndarray:
    """Turn a 1-D projection array into a vertical bar image of height h, width 80."""
    vis = np.zeros((h, 80, 3), dtype=np.uint8)
    scale = proj.max() if proj.max() > 0 else 1.0
    for i in range(min(h, len(proj))):
        bar = int(round(proj[i] / scale * 79))
        cv2.line(vis, (0, i), (bar, i), (180, 180, 180), 1)
    return vis


def _global_grid_ys(ref_y: float, cy: float, tile_h: int, T: float) -> List[float]:
    """Return all canvas-grid Y positions that fall inside this tile's local image."""
    # Local y = canvas_y - (p.cy - tile_h/2)
    offset = cy - tile_h / 2.0
    ys = []
    # Scan enough periods to cover the tile height.
    for k in range(-5, 20):
        y_local = ref_y + k * T - offset
        if -1 < y_local < tile_h + 1:
            ys.append(y_local)
    return ys


def _smooth_yaw(tiles, window: int = 3) -> None:
    yaws = np.array([t.yaw_deg for t in tiles], dtype=np.float64)
    half = window // 2
    for i in range(len(tiles)):
        lo, hi = max(0, i - half), min(len(tiles), i + half + 1)
        median = float(np.median(yaws[lo:hi]))
        if abs(yaws[i] - median) > 5.0:
            tiles[i].yaw_deg = median


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input",  required=True, type=Path)
    ap.add_argument("--output", required=True, type=Path)
    ap.add_argument("--ppm",    type=float, default=500.0)
    ap.add_argument("--fov-v-deg", type=float, default=43.0)
    ap.add_argument("--yaw-sign", type=float, default=-1.0)
    ap.add_argument("--method", default="auto",
                    choices=["auto", "projection", "derotate", "hough"])
    ap.add_argument("--only-skipped", action="store_true",
                    help="Only generate debug images for tiles whose snap was skipped.")
    args = ap.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)

    cfg = PlaceConfig(ppm=args.ppm, fov_v_deg=args.fov_v_deg, yaw_sign=args.yaw_sign)

    tiles = load_tiles(args.input)
    bgrs  = [clahe_lab(t.img) for t in tiles]

    # Apply image-yaw estimation (same as stitch_pose --use-image-yaw).
    for t, bgr in zip(tiles, bgrs):
        detected = estimate_tile_yaw(bgr)
        if detected is not None:
            t.yaw_deg = detected
    _smooth_yaw(tiles)

    extent  = world_extent(tiles)
    fp_max  = max(footprint_pixels(t.z, t.img_h, cfg.fov_v_deg, cfg.ppm) for t in tiles)
    spec    = make_canvas_spec(extent, cfg, footprint_px_max=fp_max)

    placed: List[PlacedTile] = []
    for t, bgr in zip(tiles, bgrs):
        placed.append(prepare_tile(t, cfg, preproc_bgr=bgr))
    assign_canvas_centers(tiles, spec, placed)

    # Estimate tape period.
    T = _estimate_period(placed, args.ppm)
    print(f"[debug] T={T:.1f}px ({T/args.ppm*100:.1f}cm)")

    # First pass: collect canvas phases using the chosen method.
    canvas_phases_y = []
    tile_methods    = []
    masks_derot     = []  # de-rotated masks for visualisation

    for p in placed:
        h, w = p.warped_bgr.shape[:2]
        m = _tape_mask(p.warped_bgr)

        if args.method == "auto":
            tm = "derotate" if abs(p.rotation_deg) > 3.0 else "projection"
        else:
            tm = args.method
        tile_methods.append(tm)

        if tm == "derotate":
            M = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), -p.rotation_deg, 1.0)
            m_dr = cv2.warpAffine(m, M, (w, h), flags=cv2.INTER_NEAREST)
            phase_y, _ = _tape_phase_derotate(m, p.rotation_deg, T)
        elif tm == "hough":
            m_dr = m.copy()
            phase_y, _ = _tape_phase_hough(p.warped_bgr, p.rotation_deg, T)
        else:
            m_dr = m.copy()
            phase_y = _phase_from_peaks(
                _find_tape_peaks(m.sum(axis=1).astype(np.float64), T), T)

        masks_derot.append(m_dr)
        cy_canvas = (p.cy - h / 2.0 + phase_y) % T if phase_y is not None else None
        canvas_phases_y.append(cy_canvas)

    valid_y = [v for v in canvas_phases_y if v is not None]
    ref_y   = _robust_ref_phase(valid_y, T) if valid_y else None
    max_corr = T / 3.0

    print(f"[debug] ref_y={ref_y:.1f}px, max_corr=±{max_corr:.0f}px")

    # Second pass: generate per-tile debug images.
    for i, (p, m_dr, cy_phase, tm) in enumerate(
            zip(placed, masks_derot, canvas_phases_y, tile_methods)):

        h, w = p.warped_bgr.shape[:2]
        m_orig = _tape_mask(p.warped_bgr)

        # Decide snap status.
        snapped = False
        skipped = False
        delta   = None
        if ref_y is not None and cy_phase is not None:
            delta = cy_phase - ref_y
            if delta > T / 2:  delta -= T
            if delta < -T / 2: delta += T
            if abs(delta) <= max_corr:
                snapped = True
            else:
                skipped = True

        if args.only_skipped and not skipped:
            continue

        # --- Panel 1: warped BGR with detected peaks + reference grid ---
        bgr_vis = p.warped_bgr.copy()

        # Global reference grid expected positions in this tile's local coords
        # (using the tile's CURRENT cy, before correction).
        if ref_y is not None:
            ref_ys_local = _global_grid_ys(ref_y, p.cy, h, T)
            _draw_hlines(bgr_vis, ref_ys_local, _ORANGE, 2)  # orange = global grid

        # Detected peaks from the de-rotated mask (what the method actually found).
        m_for_peaks = m_dr
        proj_y = m_for_peaks.sum(axis=1).astype(np.float64)
        peaks_y = _find_tape_peaks(proj_y, T)
        _draw_hlines(bgr_vis, peaks_y, _CYAN, 2)             # cyan = detected

        # Annotate.
        status = (f"SKIP dy={delta:+.0f}px" if skipped
                  else f"snap dy={delta:+.1f}px" if snapped
                  else "no phase")
        cv2.putText(bgr_vis, f"wp_{p.idx:02d} rot={p.rotation_deg:+.1f}deg [{tm}]",
                    (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, _WHITE, 2)
        cv2.putText(bgr_vis, status,
                    (6, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    _RED if skipped else _GREEN, 2)

        # --- Panel 2: de-rotated tape mask with same overlays ---
        mask_vis = cv2.cvtColor(m_dr, cv2.COLOR_GRAY2BGR)
        if ref_y is not None:
            _draw_hlines(mask_vis, ref_ys_local, _ORANGE, 2)
        _draw_hlines(mask_vis, peaks_y, _CYAN, 2)

        # --- Panel 3: Y-projection bar chart ---
        proj_vis = _projection_image(proj_y, h)
        # Draw reference grid lines on projection.
        if ref_y is not None:
            _draw_hlines(proj_vis, ref_ys_local, _ORANGE, 1)
        # Draw detected peaks.
        _draw_hlines(proj_vis, peaks_y, _CYAN, 1)
        # Draw canvas phase of this tile (green = where we measured it to be).
        if cy_phase is not None:
            y_measured_local = cy_phase - (p.cy - h / 2.0)
            # This value is phase in tile-local coords based on all detected peaks.
            # Not directly a single peak position, but we can show it.
            cv2.putText(proj_vis, "ref", (2, 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, _ORANGE, 1)
            cv2.putText(proj_vis, "det", (2, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, _CYAN, 1)

        # Pad panels to the same height and stack horizontally.
        panel_h = h
        panels = [bgr_vis, mask_vis, proj_vis]
        # Resize all to the same height.
        composite = np.hstack(panels)

        out_path = args.output / f"dbg_grid_{p.idx:02d}.png"
        cv2.imwrite(str(out_path), composite)
        cy_str  = f"{cy_phase:.1f}" if cy_phase is not None else "n/a"
        ref_str = f"{ref_y:.1f}"   if ref_y is not None    else "n/a"
        dlt_str = f"{delta:+.0f}px" if delta is not None   else "n/a"
        print(f"  wp_{p.idx:02d}: method={tm}, cy_phase={cy_str}, "
              f"ref_y={ref_str}, delta={dlt_str}")

    print(f"[debug] Done. Images in {args.output}")
    print(f"\nLEGEND: CYAN=detected tape peaks, ORANGE=global reference grid")


if __name__ == "__main__":
    main()
