"""RGB feather blending + per-mask weighted voting -> occupancy map (Guide §4.4 + §9)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
import yaml

from .place_pose import PlacedTile, _rotate_full, _radial_feather, footprint_pixels, PlaceConfig
from .io_utils import Tile
from .preprocess import grid_mask, obstacle_mask, wall_mask, valid_mask


# Label codes for the occupancy/semantic map.
L_UNKNOWN = 0
L_FREE = 1
L_OCCUPIED = 2
L_WALL = 3
L_GRID = 4


@dataclass
class VoteAccum:
    """Per-pixel weighted accumulators over the canvas."""
    free: np.ndarray
    occupied: np.ndarray
    wall: np.ndarray
    grid: np.ndarray
    weight: np.ndarray


def _rotate_local_mask(mask: np.ndarray, angle_deg: float) -> np.ndarray:
    return _rotate_full(mask, angle_deg, cv2.INTER_NEAREST, border_value=0)


def _rotate_local_float(buf: np.ndarray, angle_deg: float) -> np.ndarray:
    return _rotate_full(buf, angle_deg, cv2.INTER_LINEAR, border_value=0.0)


def _paste_add(
    target: np.ndarray, src: np.ndarray, cx: float, cy: float
) -> None:
    H, W = target.shape[:2]
    h, w = src.shape[:2]
    x0 = int(round(cx - w / 2.0))
    y0 = int(round(cy - h / 2.0))
    sx0 = max(0, -x0); sy0 = max(0, -y0)
    dx0 = max(0, x0);  dy0 = max(0, y0)
    dx1 = min(W, x0 + w); dy1 = min(H, y0 + h)
    if dx1 <= dx0 or dy1 <= dy0:
        return
    sw = dx1 - dx0; sh = dy1 - dy0
    target[dy0:dy1, dx0:dx1] += src[sy0:sy0 + sh, sx0:sx0 + sw]


def _paste_argmax_tile_id(
    target: np.ndarray, weight_top: np.ndarray, src_w: np.ndarray, tile_id: int, cx: float, cy: float
) -> None:
    """Where src_w > weight_top, set target=tile_id and weight_top=src_w."""
    H, W = target.shape[:2]
    h, w = src_w.shape
    x0 = int(round(cx - w / 2.0)); y0 = int(round(cy - h / 2.0))
    sx0 = max(0, -x0); sy0 = max(0, -y0)
    dx0 = max(0, x0);  dy0 = max(0, y0)
    dx1 = min(W, x0 + w); dy1 = min(H, y0 + h)
    if dx1 <= dx0 or dy1 <= dy0:
        return
    sw = dx1 - dx0; sh = dy1 - dy0
    src_view = src_w[sy0:sy0 + sh, sx0:sx0 + sw]
    top_view = weight_top[dy0:dy1, dx0:dx1]
    update = src_view > top_view
    weight_top[dy0:dy1, dx0:dx1] = np.where(update, src_view, top_view)
    tgt_view = target[dy0:dy1, dx0:dx1]
    target[dy0:dy1, dx0:dx1] = np.where(update, tile_id, tgt_view)


def blend_and_vote(
    tiles: List[Tile],
    placed: List[PlacedTile],
    cfg: PlaceConfig,
    canvas_h: int,
    canvas_w: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Render final mosaic + per-class label probabilities.

    Returns (mosaic_bgr_uint8, label_map_uint8, seam_map_uint8, prob_stack).
    prob_stack: float32 (4, H, W) for (free, occupied, wall, grid) probabilities.
    """
    accum_rgb = np.zeros((canvas_h, canvas_w, 3), dtype=np.float32)
    weight = np.zeros((canvas_h, canvas_w), dtype=np.float32)

    free_w = np.zeros((canvas_h, canvas_w), dtype=np.float32)
    occ_w = np.zeros((canvas_h, canvas_w), dtype=np.float32)
    wall_w = np.zeros((canvas_h, canvas_w), dtype=np.float32)
    grid_w = np.zeros((canvas_h, canvas_w), dtype=np.float32)

    seam = np.full((canvas_h, canvas_w), -1, dtype=np.int16)
    seam_top = np.zeros((canvas_h, canvas_w), dtype=np.float32)

    tile_by_idx = {t.idx: t for t in tiles}

    for p in placed:
        tile = tile_by_idx[p.idx]

        # Recompute per-tile resized + rotated layers (BGR + masks).
        fp = footprint_pixels(tile.z, tile.img_h, cfg.fov_v_deg, cfg.ppm)
        scale = fp / float(tile.img_h)
        new_w = max(8, int(round(tile.img_w * scale)))
        new_h = fp
        bgr = cv2.resize(tile.img, (new_w, new_h), interpolation=cv2.INTER_AREA)

        vm = valid_mask(bgr)
        gm = grid_mask(bgr)
        om = obstacle_mask(bgr)
        wm = wall_mask(bgr)

        feather = _radial_feather(new_h, new_w) * (vm.astype(np.float32) / 255.0)

        angle = cfg.yaw_sign * tile.yaw_deg + cfg.yaw_axis_offset_deg
        bgr_r = _rotate_local_float(bgr.astype(np.float32), angle)
        feather_r = _rotate_local_float(feather, angle)
        vm_r = _rotate_local_mask(vm, angle)
        gm_r = _rotate_local_mask(gm, angle)
        om_r = _rotate_local_mask(om, angle)
        wm_r = _rotate_local_mask(wm, angle)

        # Free-floor = valid AND NOT (grid OR obstacle OR wall).
        free_local = (vm_r > 0) & (gm_r == 0) & (om_r == 0) & (wm_r == 0)

        # Weighted vote contributions, scaled by feather and by 1/z (closer tile = higher weight).
        z_w = 1.0 / max(tile.z, 1e-3)

        # RGB blend.
        _paste_add(accum_rgb, bgr_r * feather_r[..., None], p.cx, p.cy)
        _paste_add(weight, feather_r, p.cx, p.cy)

        # Mask votes.
        base_w = feather_r * z_w
        _paste_add(free_w, base_w * free_local.astype(np.float32), p.cx, p.cy)
        _paste_add(occ_w,  base_w * (om_r > 0).astype(np.float32), p.cx, p.cy)
        _paste_add(wall_w, base_w * (wm_r > 0).astype(np.float32), p.cx, p.cy)
        _paste_add(grid_w, base_w * (gm_r > 0).astype(np.float32), p.cx, p.cy)

        # Seam map: tile id with the highest feather weight at each pixel.
        _paste_argmax_tile_id(seam, seam_top, feather_r, tile.idx, p.cx, p.cy)

    mosaic = accum_rgb / np.maximum(weight[..., None], 1e-6)
    mosaic = mosaic.clip(0, 255).astype(np.uint8)

    total = free_w + occ_w + wall_w + grid_w
    safe = np.maximum(total, 1e-6)
    P_free = free_w / safe
    P_occ = occ_w / safe
    P_wall = wall_w / safe
    P_grid = grid_w / safe

    label = np.full((canvas_h, canvas_w), L_UNKNOWN, dtype=np.uint8)
    observed = total > 1e-6
    max_p = np.maximum.reduce([P_free, P_occ, P_wall, P_grid])
    confident = observed & (max_p >= 0.3)

    label[confident & (P_occ == max_p)] = L_OCCUPIED
    label[confident & (P_wall == max_p)] = L_WALL
    label[confident & (P_free == max_p)] = L_FREE
    label[confident & (P_grid == max_p)] = L_GRID

    # Morphological cleanup on occupied + wall.
    occ_bin = (label == L_OCCUPIED).astype(np.uint8) * 255
    occ_bin = cv2.morphologyEx(occ_bin, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    occ_bin = cv2.morphologyEx(occ_bin, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    # Drop small CCs.
    num, lbls, stats, _ = cv2.connectedComponentsWithStats(occ_bin, connectivity=8)
    cleaned = np.zeros_like(occ_bin)
    for k in range(1, num):
        if stats[k, cv2.CC_STAT_AREA] >= 50:
            cleaned[lbls == k] = 255

    # Wall close.
    wall_bin = (label == L_WALL).astype(np.uint8) * 255
    wall_bin = cv2.morphologyEx(wall_bin, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    # Reassemble cleaned label.
    label_clean = label.copy()
    label_clean[label == L_OCCUPIED] = L_FREE  # reset occupied first
    label_clean[cleaned > 0] = L_OCCUPIED
    label_clean[wall_bin > 0] = L_WALL

    prob_stack = np.stack([P_free, P_occ, P_wall, P_grid], axis=0).astype(np.float32)
    seam_vis = _colorize_seam(seam)
    return mosaic, label_clean, seam_vis, prob_stack


def _colorize_seam(seam: np.ndarray) -> np.ndarray:
    """Map int16 tile ids to a distinct color per id; -1 stays black."""
    H, W = seam.shape
    out = np.zeros((H, W, 3), dtype=np.uint8)
    palette = (np.array([
        [0xE6, 0x19, 0x4B], [0x3C, 0xB4, 0x4B], [0xFF, 0xE1, 0x19], [0x43, 0x63, 0xD8],
        [0xF5, 0x82, 0x31], [0x91, 0x1E, 0xB4], [0x42, 0xD4, 0xF4], [0xF0, 0x32, 0xE6],
        [0xBF, 0xEF, 0x45], [0xFA, 0xBE, 0xD4], [0x46, 0x99, 0x90], [0xDC, 0xBE, 0xFF],
        [0x9A, 0x63, 0x24], [0xFF, 0xFA, 0xC8], [0x80, 0x00, 0x00], [0xAA, 0xFF, 0xC3],
    ], dtype=np.uint8))
    for tid in range(16):
        out[seam == tid] = palette[tid]
    return out


def label_to_ros_pgm(label: np.ndarray) -> np.ndarray:
    """Convert internal label map into a ROS occupancy_grid PGM image.

    ROS map_server convention (trinary):
        0   (black) -> occupied
        205 (gray)  -> unknown
        254 (white) -> free
    """
    out = np.full(label.shape, 205, dtype=np.uint8)  # unknown
    out[label == L_FREE] = 254
    out[label == L_GRID] = 254  # treat orange grid as free for path planning
    out[label == L_OCCUPIED] = 0
    out[label == L_WALL] = 0
    return out


def write_ros_map(
    pgm_image: np.ndarray,
    out_dir: Path,
    name: str,
    resolution_m_per_px: float,
    origin_xy: Tuple[float, float] = (0.0, 0.0),
) -> Tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = out_dir / f"{name}.pgm"
    yaml_path = out_dir / f"{name}.yaml"
    cv2.imwrite(str(pgm_path), pgm_image)
    with open(yaml_path, "w") as f:
        yaml.dump({
            "image": pgm_path.name,
            "mode": "trinary",
            "resolution": float(resolution_m_per_px),
            "origin": [float(origin_xy[0]), float(origin_xy[1]), 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }, f)
    return pgm_path, yaml_path
