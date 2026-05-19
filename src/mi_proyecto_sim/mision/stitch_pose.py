"""End-to-end pose-aware stitcher.

Usage:
    python3 -m mision.stitch_pose --input mision_output/16fotos \
                                  --output mision_output/stitching_pose \
                                  --debug
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from .io_utils import load_tiles, world_extent
from .place_pose import (
    PlaceConfig, footprint_pixels, make_canvas_spec,
    prepare_tile, assign_canvas_centers, paste_phase_a, annotate_indices,
    tile_layout_debug,
)
from .preprocess import clahe_lab, edge_image, grid_mask, estimate_tile_yaw
from .refine_window import refine_all, RefineConfig
from .solve_global import solve_positions, apply_positions, summarize_residuals
from .blend_vote import blend_and_vote, label_to_ros_pgm, label_to_grid_vis, write_ros_map, mask_exterior, build_final_label, crop_to_arena
from .tape_snap import snap_to_tape_grid


def _smooth_yaw(tiles, window: int = 3) -> None:
    """Median-filter detected yaw across tile index order; replaces only outliers."""
    yaws = np.array([t.yaw_deg for t in tiles], dtype=np.float64)
    half = window // 2
    for i in range(len(tiles)):
        lo, hi = max(0, i - half), min(len(tiles), i + half + 1)
        median = float(np.median(yaws[lo:hi]))
        if abs(yaws[i] - median) > 5.0:
            tiles[i].yaw_deg = median


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, type=Path)
    ap.add_argument("--output", required=True, type=Path)
    ap.add_argument("--ppm", type=float, default=500.0)
    ap.add_argument("--fov-v-deg", type=float, default=43.0)
    ap.add_argument("--yaw-sign", type=float, default=1.0,
                    help="Per-tile yaw multiplier applied to estimate_tile_yaw output. "
                         "+1 (default) correctly corrects the drone heading tilt. "
                         "-1 doubles the tilt instead of removing it.")
    ap.add_argument("--yaw-axis-offset-deg", type=float, default=0.0)
    ap.add_argument("--use-image-yaw", action="store_true",
                    help="Estimate per-tile yaw from orange tape grid line angle. "
                         "Replaces drifting Optitrack heading with image-derived heading; "
                         "falls back to Optitrack yaw when detection confidence is low.")
    ap.add_argument("--yaw-detrend", action="store_true",
                    help="Replace each tile's yaw with the dataset mean yaw. "
                         "Cancels monotonic heading drift (e.g. Optitrack) while "
                         "preserving the mean orientation of the camera.")
    ap.add_argument("--tape-snap", action="store_true",
                    help="After Phase A, shift each tile so its tape lines align to a "
                         "single global 90° grid detected from the images. Removes "
                         "sub-tape-period position errors without phase-correlation aliasing.")
    ap.add_argument("--tape-period-m", type=float, default=None,
                    help="Tape grid period in metres (default: auto-estimated from images).")
    ap.add_argument("--max-correction-frac", type=float, default=0.47,
                    help="Max tape-snap correction as a fraction of tape period T "
                         "(default 0.47 = T/2 - 8px safety margin). "
                         "Old value was 0.333; increase to fix skipped tiles with large errors.")
    ap.add_argument("--snap-method", default="auto",
                    choices=["auto", "projection", "derotate", "hough"],
                    help="Tape-snap phase-detection method. "
                         "'auto' uses derotate for |rotation|>3°, projection otherwise. "
                         "'derotate' rotates the mask to make tape lines axis-aligned. "
                         "'hough' uses HoughLines intercepts at tile centre (native angle support).")
    ap.add_argument("--search-radius-px", type=int, default=60)
    ap.add_argument("--skip-refine", action="store_true",
                    help="Use pose-prior placement only; skip Phase B/C.")
    ap.add_argument("--seam-width", type=int, default=25,
                    help="seam_cut blend transition width in pixels (default 25). "
                         "Wider = smoother tape-line kinks from residual snap noise; "
                         "narrower = harder transitions.")
    ap.add_argument("--blend-mode", default="feather",
                    choices=["feather", "gain", "multiband", "gain+multiband", "seam_cut", "strip_seam_cut"],
                    help="RGB blending strategy. "
                         "'feather': radial-Gaussian weighted average (default). "
                         "'gain': per-tile brightness normalisation then feather. "
                         "'multiband': 4-level Laplacian-pyramid blend. "
                         "'gain+multiband': both gain comp and multiband. "
                         "'seam_cut': winner-takes-all + 15px boundary blend + gain comp.")
    ap.add_argument("--occ-min-frac", type=float, default=0.12,
                    help="Fraction of tiles that must detect a canvas pixel as obstacle "
                         "(default 0.12 = 12%%). Scales automatically with tile count: "
                         "16 tiles → min 2 votes, 55 tiles → min 7 votes.")
    ap.add_argument("--occ-close-px", type=int, default=12,
                    help="Morphological closing radius for obstacle mask (px). "
                         "Fills within-object detection gaps without bridging between "
                         "separate objects. At 500px/m: 12px ≈ 2.4cm (default).")
    ap.add_argument("--arena-size-m", type=float, default=None,
                    help="Fixed arena canvas size in metres (e.g. 3.9 for a 3.9×3.9 m arena). "
                         "Crops/pads the final map and mosaic to a square of this size, "
                         "centred on the tape-grid centroid. Omit to keep the full canvas.")
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--map-name", default="occupancy_map")
    args = ap.parse_args()

    out = args.output
    out.mkdir(parents=True, exist_ok=True)

    cfg = PlaceConfig(
        ppm=args.ppm,
        fov_v_deg=args.fov_v_deg,
        yaw_sign=args.yaw_sign,
        yaw_axis_offset_deg=args.yaw_axis_offset_deg,
    )

    # 1) Load.
    tiles = load_tiles(args.input)
    print(f"[stitch_pose] {len(tiles)} tiles, "
          f"yaw range [{min(t.yaw_deg for t in tiles):+.2f}°, {max(t.yaw_deg for t in tiles):+.2f}°], "
          f"z range [{min(t.z for t in tiles):.3f}, {max(t.z for t in tiles):.3f}]m")

    # 2) Canvas.
    extent = world_extent(tiles)
    fp_max = max(footprint_pixels(t.z, t.img_h, cfg.fov_v_deg, cfg.ppm) for t in tiles)
    spec = make_canvas_spec(extent, cfg, footprint_px_max=fp_max)
    print(f"[stitch_pose] Canvas {spec.width}x{spec.height} px @ {cfg.ppm} px/m")

    # 3) Phase A: pose-prior placement.
    # Pre-compute CLAHE images (reused for yaw estimation and prepare_tile).
    bgrs = [clahe_lab(t.img) for t in tiles]

    if args.use_image_yaw:
        # Pass 1: detect per-tile yaw from grid lines, update tile.yaw_deg.
        n_detected = 0
        for t, bgr in zip(tiles, bgrs):
            orig = t.yaw_deg
            detected = estimate_tile_yaw(bgr)
            if detected is not None:
                t.yaw_deg = detected
                n_detected += 1
                if args.debug:
                    print(f"  wp_{t.idx:02d}: optitrack={orig:+.2f}° → image={t.yaw_deg:+.2f}°")
        # Smooth: replace outliers (>5° from local median) with the median.
        _smooth_yaw(tiles)
        print(f"[stitch_pose] --use-image-yaw: {n_detected}/{len(tiles)} tiles used image yaw")

    if args.yaw_detrend:
        # Snap all tiles to the mean yaw (after image yaw detection if active).
        # Removes per-tile variation so Phase B sees consistent overlap geometry,
        # while preserving the true global arena tilt detected from the images.
        mean_yaw = sum(t.yaw_deg for t in tiles) / len(tiles)
        for t in tiles:
            t.yaw_deg = mean_yaw
        print(f"[stitch_pose] --yaw-detrend: all tile yaws set to mean {mean_yaw:+.2f}°")

    # Pass 2: warp tiles with the (possibly updated) yaw values.
    placed = []
    for t, bgr in zip(tiles, bgrs):
        placed.append(prepare_tile(t, cfg, preproc_bgr=bgr))
    assign_canvas_centers(tiles, spec, placed)

    # Phase A+: tape-grid snap — shift each tile so its tape lines align to a
    # single global 90° reference grid detected from the images.
    if args.tape_snap:
        n_snapped = snap_to_tape_grid(
            placed, ppm=cfg.ppm,
            tape_period_m=args.tape_period_m,
            max_correction_frac=args.max_correction_frac,
            method=args.snap_method,
            debug=args.debug,
        )
        print(f"[stitch_pose] --tape-snap: {n_snapped}/{len(placed)} tiles snapped to tape grid")

    if args.debug:
        layout = tile_layout_debug(tiles, placed, cfg, spec.height, spec.width)
        cv2.imwrite(str(out / "debug_tile_layout.png"), layout)

    # 4) Phase B: sliding-window refinement.
    if not args.skip_refine:
        rcfg = RefineConfig(search_radius_px=args.search_radius_px)
        print(f"[stitch_pose] Refining {len(placed)} tiles with phaseCorrelate "
              f"(±{rcfg.search_radius_px}px search)...")
        # Grayscale of CLAHE image: includes both tape lines and aperiodic floor
        # content (objects, shadows), which helps break grid periodicity.
        def _gray_func(bgr: np.ndarray) -> np.ndarray:
            return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        pairs = refine_all(spec.height, spec.width, placed, _gray_func, rcfg)
        accepted = [p for p in pairs if p.accepted]
        print(f"[stitch_pose] Phase B: {len(accepted)}/{len(pairs)} pairs accepted "
              f"(min_score={rcfg.min_score})")
        if args.debug:
            with open(out / "debug_pairs.txt", "w") as f:
                for pr in pairs:
                    f.write(
                        f"({pr.i:02d},{pr.j:02d})  "
                        f"prior=({pr.dx_prior:+.1f},{pr.dy_prior:+.1f})  "
                        f"refined=({pr.dx_refined:+.1f},{pr.dy_refined:+.1f})  "
                        f"delta=({pr.dx_refined - pr.dx_prior:+.1f},{pr.dy_refined - pr.dy_prior:+.1f})  "
                        f"score={pr.score:+.4f}  {'OK' if pr.accepted else 'REJECTED'}\n"
                    )

        # 5) Phase C: global solve.
        if accepted:
            positions = solve_positions(placed, pairs, anchor_idx=0)
            print(f"[stitch_pose] Phase C: {summarize_residuals(pairs, positions)}")
            apply_positions(placed, positions)
        else:
            print("[stitch_pose] WARN: no accepted pairs — falling back to pose-only placement")

    # 6) Blending + voting.
    print(f"[stitch_pose] Blending ({args.blend_mode}) + per-mask voting...")
    mosaic, label, seam_vis, prob_stack, occ_count, coverage_count = blend_and_vote(
        tiles, placed, cfg, spec.height, spec.width,
        blend_mode=args.blend_mode, seam_width_px=args.seam_width,
    )
    label, mosaic, interior_mask = mask_exterior(label, mosaic)
    n_interior = int(interior_mask.sum())
    n_total = interior_mask.size
    print(f"[stitch_pose] mask_exterior: {100*n_interior/n_total:.1f}% interior, "
          f"{100*(n_total-n_interior)/n_total:.1f}% exterior masked")

    # Build final occupancy map from direct color detection on the blended mosaic.
    # Coverage-normalized vote fraction: corners/edges get fair treatment.
    print(f"[stitch_pose] occ_min_frac={args.occ_min_frac:.0%} (coverage-normalized per pixel)")
    label = build_final_label(mosaic, interior_mask, label_grid_ref=label,
                              occ_count=occ_count,
                              coverage_count=coverage_count,
                              occ_min_frac=args.occ_min_frac,
                              occ_close_px=args.occ_close_px)
    print("[stitch_pose] build_final_label: occupancy map rebuilt from mosaic colour")

    # Optional: crop both mosaic and label to a fixed arena square.
    crop_offset = (0, 0)
    if args.arena_size_m is not None:
        label, mosaic, crop_offset = crop_to_arena(label, mosaic, args.arena_size_m, cfg.ppm)
        arena_px = int(round(args.arena_size_m * cfg.ppm))
        print(f"[stitch_pose] crop_to_arena: {arena_px}×{arena_px} px "
              f"({args.arena_size_m}m × {args.arena_size_m}m)")

    if args.debug:
        canvas_a, _ = paste_phase_a(spec, placed)
        canvas_a[~interior_mask] = 0
        cv2.imwrite(str(out / "debug_phase_a.png"), annotate_indices(canvas_a, placed))

    label_stamp = f"blend={args.blend_mode}"
    cv2.putText(mosaic, label_stamp, (12, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(mosaic, label_stamp, (12, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 0), 2, cv2.LINE_AA)
    cv2.imwrite(str(out / "mosaic_pose.png"), mosaic)
    cv2.imwrite(str(out / "mosaic_seams.png"), seam_vis)

    # 7) ROS map.
    pgm_img = label_to_ros_pgm(label)
    # Origin in world coordinates: the canvas origin (top-left) corresponds to world (x_min - margin/ppm, y_max + margin/ppm).
    # ROS uses bottom-left origin, so we flip the image vertically and compute origin accordingly.
    pgm_flipped = cv2.flip(pgm_img, 0)
    resolution = 1.0 / cfg.ppm
    # Canvas top-left in world coords (adjusted for crop offset if arena crop was applied):
    world_x_left = extent.x_min - cfg.margin_px / cfg.ppm + crop_offset[0] / cfg.ppm
    world_y_top = extent.y_max + cfg.margin_px / cfg.ppm - crop_offset[1] / cfg.ppm
    # Bottom-left after vflip:
    world_y_bottom = world_y_top - pgm_img.shape[0] / cfg.ppm
    write_ros_map(
        pgm_flipped, out, args.map_name, resolution,
        origin_xy=(world_x_left, world_y_bottom),
    )
    cv2.imwrite(str(out / (args.map_name + "_vis.png")), pgm_img)
    tape_period_px = 0.53 * cfg.ppm  # 48cm tile + 5cm tape = 53cm
    grid_vis = label_to_grid_vis(label, tape_period_px=tape_period_px)
    cv2.imwrite(str(out / (args.map_name + "_grid.png")), grid_vis)

    # Overlay: mosaic at 50% alpha over the binary map.
    map_bgr = cv2.cvtColor(pgm_img, cv2.COLOR_GRAY2BGR)
    overlay = cv2.addWeighted(map_bgr, 0.5, mosaic, 0.5, 0)
    cv2.imwrite(str(out / (args.map_name + "_overlay.png")), overlay)
    print(f"[stitch_pose] Wrote {out/(args.map_name + '.pgm')} ({pgm_img.shape[1]}x{pgm_img.shape[0]}, "
          f"res={resolution:.4f} m/px)")

    # Optional landmark layer (grid only).
    if args.debug:
        grid_layer = (prob_stack[3] * 255).clip(0, 255).astype(np.uint8)
        cv2.imwrite(str(out / "landmark_grid.png"), grid_layer)

    print(f"[stitch_pose] Done. Outputs in {out}")


if __name__ == "__main__":
    main()
