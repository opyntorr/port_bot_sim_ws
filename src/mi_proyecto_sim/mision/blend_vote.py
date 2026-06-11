"""RGB feather blending + per-mask weighted voting -> occupancy map (Guide §4.4 + §9)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Dict, Optional

import cv2
import numpy as np
import yaml

from .place_pose import PlacedTile, _rotate_full, _radial_feather, footprint_pixels, PlaceConfig
from .io_utils import Tile
from .preprocess import grid_mask, obstacle_mask, wall_mask, valid_mask, marker_mask


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


def _paste_or(target: np.ndarray, src: np.ndarray, cx: float, cy: float) -> None:
    H, W = target.shape[:2]
    h, w = src.shape[:2]
    x0 = int(round(cx - w / 2.0)); y0 = int(round(cy - h / 2.0))
    sx0 = max(0, -x0); sy0 = max(0, -y0)
    dx0 = max(0, x0);  dy0 = max(0, y0)
    dx1 = min(W, x0 + w); dy1 = min(H, y0 + h)
    if dx1 <= dx0 or dy1 <= dy0:
        return
    sw = dx1 - dx0; sh = dy1 - dy0
    target[dy0:dy1, dx0:dx1] |= src[sy0:sy0 + sh, sx0:sx0 + sw]


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


def _compute_gains(
    tiles: List[Tile],
    placed: List[PlacedTile],
    cfg: PlaceConfig,
) -> Dict[int, float]:
    """Per-tile multiplicative gain to normalise brightness to dataset median."""
    tile_by_idx = {t.idx: t for t in tiles}
    means: Dict[int, float] = {}
    for p in placed:
        tile = tile_by_idx[p.idx]
        fp = footprint_pixels(tile.z, tile.img_h, cfg.fov_v_deg, cfg.ppm)
        scale = fp / float(tile.img_h)
        bgr = cv2.resize(tile.img, (max(8, int(round(tile.img_w * scale))), fp),
                         interpolation=cv2.INTER_AREA)
        vm = valid_mask(bgr)
        if int(vm.sum()) > 100 * 255:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            means[p.idx] = float(gray[vm > 0].mean())
    if not means:
        return {p.idx: 1.0 for p in placed}
    target = float(np.median(list(means.values())))
    gains: Dict[int, float] = {}
    for p in placed:
        m = means.get(p.idx)
        gains[p.idx] = 1.0 if (m is None or m < 1.0) else float(np.clip(target / m, 0.5, 2.0))
    return gains


def _paste_multiband(
    mb_accum: List[np.ndarray],
    mb_weight: List[np.ndarray],
    bgr_r: np.ndarray,
    feather_r: np.ndarray,
    cx: float,
    cy: float,
    n_levels: int,
) -> None:
    """Paste tile Laplacian pyramid into canvas-level pyramid accumulators."""
    gpyr_bgr: List[np.ndarray] = [bgr_r.copy()]
    for _ in range(n_levels):
        gpyr_bgr.append(cv2.pyrDown(gpyr_bgr[-1]))

    lpyr_bgr: List[np.ndarray] = []
    for L in range(n_levels):
        h, w = gpyr_bgr[L].shape[:2]
        lpyr_bgr.append(gpyr_bgr[L] - cv2.pyrUp(gpyr_bgr[L + 1], dstsize=(w, h)))
    lpyr_bgr.append(gpyr_bgr[n_levels])  # coarsest level is the Gaussian itself

    gpyr_f: List[np.ndarray] = [feather_r.copy()]
    for _ in range(n_levels):
        gpyr_f.append(cv2.pyrDown(gpyr_f[-1]))

    for L in range(n_levels + 1):
        cx_L = cx / float(1 << L)
        cy_L = cy / float(1 << L)
        f_L = gpyr_f[L].astype(np.float64)
        lap_L = lpyr_bgr[L].astype(np.float64)
        _paste_add(mb_accum[L], lap_L * f_L[..., None], cx_L, cy_L)
        _paste_add(mb_weight[L], f_L, cx_L, cy_L)


def _reconstruct_multiband(
    mb_accum: List[np.ndarray],
    mb_weight: List[np.ndarray],
    canvas_h: int,
    canvas_w: int,
    n_levels: int,
) -> np.ndarray:
    """Normalise pyramid levels and reconstruct the mosaic."""
    blend_pyr = [
        (mb_accum[L] / np.maximum(mb_weight[L][..., None], 1e-6)).astype(np.float32)
        for L in range(n_levels + 1)
    ]
    result = blend_pyr[n_levels]
    for L in range(n_levels - 1, -1, -1):
        h, w = blend_pyr[L].shape[:2]
        result = cv2.pyrUp(result, dstsize=(w, h)) + blend_pyr[L]
    return result.clip(0, 255).astype(np.uint8)


def _build_seam_map(
    tiles: List[Tile],
    placed: List[PlacedTile],
    cfg: PlaceConfig,
    canvas_h: int,
    canvas_w: int,
) -> np.ndarray:
    """Lightweight pass: argmax-feather seam map (which tile owns each pixel)."""
    tile_by_idx = {t.idx: t for t in tiles}
    seam = np.full((canvas_h, canvas_w), -1, dtype=np.int16)
    seam_top = np.zeros((canvas_h, canvas_w), dtype=np.float32)
    for p in placed:
        tile = tile_by_idx[p.idx]
        fp = footprint_pixels(tile.z, tile.img_h, cfg.fov_v_deg, cfg.ppm)
        scale = fp / float(tile.img_h)
        new_w = max(8, int(round(tile.img_w * scale)))
        bgr = cv2.resize(tile.img, (new_w, fp), interpolation=cv2.INTER_AREA)
        vm = valid_mask(bgr)
        feather = _radial_feather(fp, new_w) * (vm.astype(np.float32) / 255.0)
        angle = cfg.yaw_sign * tile.yaw_deg + cfg.yaw_axis_offset_deg
        feather_r = _rotate_local_float(feather, angle)
        _paste_argmax_tile_id(seam, seam_top, feather_r, tile.idx, p.cx, p.cy)
    return seam


def _paste_seam_cut(
    accum_rgb: np.ndarray,
    weight: np.ndarray,
    bgr_r: np.ndarray,
    vm_r: np.ndarray,
    tile_idx: int,
    cx: float,
    cy: float,
    seam: np.ndarray,
    seam_width_px: int = 15,
) -> None:
    """Winner-takes-all seam paste: each pixel from its dominant tile, narrow Gaussian
    blend (seam_width_px) only at tile boundaries. Zero ghosting in non-boundary zones."""
    H, W = accum_rgb.shape[:2]
    h, w = bgr_r.shape[:2]
    x0 = int(round(cx - w / 2.0)); y0 = int(round(cy - h / 2.0))
    x1, y1 = x0 + w, y0 + h

    PAD = seam_width_px * 4
    px0 = max(0, x0 - PAD); py0 = max(0, y0 - PAD)
    px1 = min(W, x1 + PAD); py1 = min(H, y1 + PAD)

    hard = (seam[py0:py1, px0:px1] == tile_idx).astype(np.float32)
    ksize = seam_width_px * 6 + 1
    soft = cv2.GaussianBlur(hard, (ksize, ksize), seam_width_px / 2.5)

    sy0 = max(0, -y0); sx0 = max(0, -x0)
    dy0 = max(0, y0); dy1 = min(H, y1)
    dx0 = max(0, x0); dx1 = min(W, x1)
    if dy1 <= dy0 or dx1 <= dx0:
        return
    sh = dy1 - dy0; sw = dx1 - dx0

    alpha = soft[dy0 - py0: dy0 - py0 + sh, dx0 - px0: dx0 - px0 + sw]
    alpha = alpha * (vm_r[sy0:sy0 + sh, sx0:sx0 + sw] > 0).astype(np.float32)

    accum_rgb[dy0:dy1, dx0:dx1] += bgr_r[sy0:sy0 + sh, sx0:sx0 + sw] * alpha[..., None]
    weight[dy0:dy1, dx0:dx1] += alpha


def _cluster_strips(
    tiles: List[Tile],
    placed: List[PlacedTile],
    gap_factor: float = 2.0,
) -> List[List[int]]:
    """Group tile indices into lawnmower strips via position gap detection.

    Tries world X first, then Y; picks the axis with more detected gaps.
    Each gap larger than gap_factor × median-gap marks a new strip boundary.
    """
    tile_by_idx = {t.idx: t for t in tiles}

    def _try(coord_fn):
        coords = np.array([coord_fn(tile_by_idx[p.idx]) for p in placed])
        order = np.argsort(coords)
        s_placed = [placed[i] for i in order]
        s_coords = coords[order]
        diffs = np.abs(np.diff(s_coords))
        if len(diffs) == 0:
            return [[p.idx for p in placed]], 0
        thresh = np.median(diffs) * gap_factor
        groups: List[List[int]] = [[s_placed[0].idx]]
        for i, d in enumerate(diffs):
            if d > thresh:
                groups.append([])
            groups[-1].append(s_placed[i + 1].idx)
        return groups, len(groups) - 1

    gx, nx = _try(lambda t: t.x)
    gy, ny = _try(lambda t: t.y)
    if nx == 0 and ny == 0:
        return [[p.idx] for p in placed]   # no clear structure; treat each tile alone
    return gx if nx >= ny else gy


def _compute_strip_gains(
    tiles: List[Tile],
    placed: List[PlacedTile],
    cfg: PlaceConfig,
    strips: List[List[int]],
) -> Dict[int, float]:
    """Per-strip gain: normalise each strip's mean brightness to the inter-strip median."""
    tile_by_idx = {t.idx: t for t in tiles}
    strip_means: Dict[int, float] = {}
    for s_i, idxs in enumerate(strips):
        vals = []
        for idx in idxs:
            tile = tile_by_idx[idx]
            fp = footprint_pixels(tile.z, tile.img_h, cfg.fov_v_deg, cfg.ppm)
            scale = fp / float(tile.img_h)
            bgr = cv2.resize(tile.img, (max(8, int(round(tile.img_w * scale))), fp),
                             interpolation=cv2.INTER_AREA)
            vm = valid_mask(bgr)
            if int(vm.sum()) > 100 * 255:
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                vals.append(float(gray[vm > 0].mean()))
        if vals:
            strip_means[s_i] = float(np.mean(vals))
    if not strip_means:
        return {s_i: 1.0 for s_i in range(len(strips))}
    target = float(np.median(list(strip_means.values())))
    return {
        s_i: 1.0 if strip_means.get(s_i, 0) < 1.0
             else float(np.clip(target / strip_means[s_i], 0.5, 2.0))
        for s_i in range(len(strips))
    }


def _paste_strip_seam_cut(
    accum_rgb: np.ndarray,
    weight: np.ndarray,
    bgr_r: np.ndarray,
    vm_r: np.ndarray,
    tile_idx: int,
    strip_idx: int,
    cx: float, cy: float,
    tile_seam: np.ndarray,
    strip_seam: np.ndarray,
    seam_width_within: int = 20,
    seam_width_between: int = 35,
) -> None:
    """Strip-aware seam cut.

    Within a lawnmower strip: narrow Gaussian blend (seam_width_within).
    At strip boundaries: wide blend (seam_width_between).
    Transition between the two zones is spatially smooth via a lerp based on
    proximity to the nearest different-strip pixel.
    """
    H, W = accum_rgb.shape[:2]
    h, w = bgr_r.shape[:2]
    x0 = int(round(cx - w / 2.0)); y0 = int(round(cy - h / 2.0))
    x1, y1 = x0 + w, y0 + h

    PAD = seam_width_between * 4
    px0 = max(0, x0 - PAD); py0 = max(0, y0 - PAD)
    px1 = min(W, x1 + PAD); py1 = min(H, y1 + PAD)

    hard = (tile_seam[py0:py1, px0:px1] == tile_idx).astype(np.float32)

    # Narrow blend (within-strip seam quality)
    kw = seam_width_within * 6 + 1
    soft_narrow = cv2.GaussianBlur(hard, (kw | 1, kw | 1), seam_width_within / 2.5)

    # Wide blend (between-strip seam quality)
    kb = seam_width_between * 6 + 1
    soft_wide = cv2.GaussianBlur(hard, (kb | 1, kb | 1), seam_width_between / 2.5)

    # Proximity to different-strip pixels → lerp weight toward wide blend
    diff_strip = (strip_seam[py0:py1, px0:px1] != strip_idx).astype(np.float32)
    near_boundary = np.clip(
        cv2.GaussianBlur(diff_strip, (kb | 1, kb | 1), seam_width_between / 2.5), 0.0, 1.0
    )

    soft = soft_narrow * (1.0 - near_boundary) + soft_wide * near_boundary

    sy0 = max(0, -y0); sx0 = max(0, -x0)
    dy0 = max(0, y0); dy1 = min(H, y1)
    dx0 = max(0, x0); dx1 = min(W, x1)
    if dy1 <= dy0 or dx1 <= dx0:
        return
    sh = dy1 - dy0; sw = dx1 - dx0

    alpha = soft[dy0 - py0: dy0 - py0 + sh, dx0 - px0: dx0 - px0 + sw]
    alpha = alpha * (vm_r[sy0:sy0 + sh, sx0:sx0 + sw] > 0).astype(np.float32)

    accum_rgb[dy0:dy1, dx0:dx1] += bgr_r[sy0:sy0 + sh, sx0:sx0 + sw] * alpha[..., None]
    weight[dy0:dy1, dx0:dx1] += alpha


def blend_and_vote(
    tiles: List[Tile],
    placed: List[PlacedTile],
    cfg: PlaceConfig,
    canvas_h: int,
    canvas_w: int,
    blend_mode: str = "feather",
    seam_width_px: int = 25,
    obstacle_height_m: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Render final mosaic + per-class label probabilities.

    blend_mode:
      "feather"          — radial Gaussian weighted average (baseline)
      "gain"             — per-tile brightness normalisation + feather
      "multiband"        — 4-level Laplacian pyramid blend
      "gain+multiband"   — both gain comp and multiband
      "seam_cut"         — winner-takes-all seam + gain comp (near-zero ghosting)
      "strip_seam_cut"   — two-level seam: narrow within lawnmower strips,
                           wide at strip boundaries + per-strip gain compensation

    Returns (mosaic_bgr_uint8, label_map_uint8, seam_map_uint8, prob_stack).
    prob_stack: float32 (4, H, W) for (free, occupied, wall, grid) probabilities.
    """
    # Exact-match flags to avoid substring collisions (e.g. "seam_cut" ⊂ "strip_seam_cut").
    use_multiband = blend_mode in ("multiband", "gain+multiband")
    use_seam_cut  = blend_mode == "seam_cut"
    use_strip     = blend_mode == "strip_seam_cut"
    use_gain      = blend_mode in ("gain", "gain+multiband") or use_seam_cut
    # strip_seam_cut uses its own strip-level gain (computed below, not use_gain)

    accum_rgb = np.zeros((canvas_h, canvas_w, 3), dtype=np.float32)
    weight    = np.zeros((canvas_h, canvas_w),    dtype=np.float32)

    free_w = np.zeros((canvas_h, canvas_w), dtype=np.float32)
    occ_w  = np.zeros((canvas_h, canvas_w), dtype=np.float32)
    wall_w = np.zeros((canvas_h, canvas_w), dtype=np.float32)
    grid_w = np.zeros((canvas_h, canvas_w), dtype=np.float32)
    occ_count     = np.zeros((canvas_h, canvas_w), dtype=np.float32)  # per-tile vote count
    coverage_count = np.zeros((canvas_h, canvas_w), dtype=np.float32)  # tiles covering each pixel

    seam     = np.full((canvas_h, canvas_w), -1, dtype=np.int16)
    seam_top = np.zeros((canvas_h, canvas_w), dtype=np.float32)

    n_levels = 4
    mb_accum: Optional[List[np.ndarray]] = None
    mb_weight: Optional[List[np.ndarray]] = None
    if use_multiband:
        mb_accum = [
            np.zeros((max(1, canvas_h >> L), max(1, canvas_w >> L), 3), dtype=np.float64)
            for L in range(n_levels + 1)
        ]
        mb_weight = [
            np.zeros((max(1, canvas_h >> L), max(1, canvas_w >> L)), dtype=np.float64)
            for L in range(n_levels + 1)
        ]

    gains: Optional[Dict[int, float]] = None
    if use_gain:
        gains = _compute_gains(tiles, placed, cfg)
        print(f"[blend] gain range [{min(gains.values()):.3f}, {max(gains.values()):.3f}]")

    # seam_cut needs the full seam map before the blend loop.
    prebuilt_seam: Optional[np.ndarray] = None
    if use_seam_cut:
        print("[blend] seam_cut: pre-building seam map...")
        prebuilt_seam = _build_seam_map(tiles, placed, cfg, canvas_h, canvas_w)

    # strip_seam_cut: cluster strips, compute per-strip gains, pre-build seam maps.
    strips = None
    strip_of: Optional[Dict[int, int]] = None
    strip_gains_map: Optional[Dict[int, float]] = None
    prebuilt_tile_seam: Optional[np.ndarray] = None
    prebuilt_strip_seam: Optional[np.ndarray] = None
    if use_strip:
        strips = _cluster_strips(tiles, placed)
        strip_of = {idx: s_i for s_i, strip in enumerate(strips) for idx in strip}
        strip_gains_map = _compute_strip_gains(tiles, placed, cfg, strips)
        n_strips = len(strips)
        gain_strs = ", ".join(f"{strip_gains_map[s]:.3f}" for s in range(n_strips))
        print(f"[blend] strip_seam_cut: {n_strips} strips, gains [{gain_strs}]")
        print("[blend] strip_seam_cut: pre-building seam maps...")
        prebuilt_tile_seam = _build_seam_map(tiles, placed, cfg, canvas_h, canvas_w)
        max_t = max(strip_of.keys()) if strip_of else 0
        lut = np.full(max_t + 2, -1, dtype=np.int16)
        for t_idx, s_i in strip_of.items():
            lut[t_idx] = s_i
        prebuilt_strip_seam = np.where(
            prebuilt_tile_seam >= 0,
            lut[np.clip(prebuilt_tile_seam.astype(np.int32), 0, max_t + 1)],
            -1,
        ).astype(np.int16)

    tile_by_idx = {t.idx: t for t in tiles}

    for p in placed:
        tile = tile_by_idx[p.idx]

        fp = footprint_pixels(tile.z, tile.img_h, cfg.fov_v_deg, cfg.ppm)
        scale = fp / float(tile.img_h)
        new_w = max(8, int(round(tile.img_w * scale)))
        new_h = fp
        bgr = cv2.resize(tile.img, (new_w, new_h), interpolation=cv2.INTER_AREA)

        if gains is not None:
            g = gains.get(p.idx, 1.0)
            bgr = np.clip(bgr.astype(np.float32) * g, 0, 255).astype(np.uint8)
        elif strip_gains_map is not None and strip_of is not None:
            g = strip_gains_map.get(strip_of.get(p.idx, 0), 1.0)
            bgr = np.clip(bgr.astype(np.float32) * g, 0, 255).astype(np.uint8)

        vm = valid_mask(bgr)
        gm = grid_mask(bgr)
        wm = wall_mask(bgr)
        mm = marker_mask(tile.img, out_hw=(new_h, new_w))
        # Run obstacle detection on the original full-res tile (before resize/gain)
        # for best colour fidelity, then scale the binary mask down.
        om_orig = obstacle_mask(tile.img)
        om = cv2.resize(om_orig, (new_w, new_h), interpolation=cv2.INTER_NEAREST)

        # Floor markers (ArUco etc.) — remove from obstacle/wall voting.
        if mm.any():
            om = cv2.bitwise_and(om, cv2.bitwise_not(mm))
            wm = cv2.bitwise_and(wm, cv2.bitwise_not(mm))

        # Ground the obstacle mask: a box of height h seen from altitude z
        # projects its top face outward from the camera nadir (image centre)
        # by z/(z-h).  Shrinking the mask radially by (z-h)/z maps the
        # top/side-face projection back onto the floor footprint, removing
        # the parallax "splay" that makes tall obstacles look oversized.
        if obstacle_height_m > 0.0 and om.any():
            s = max(tile.z - obstacle_height_m, 1e-3) / max(tile.z, 1e-3)
            M = np.float32([
                [s, 0, (1.0 - s) * new_w / 2.0],
                [0, s, (1.0 - s) * new_h / 2.0],
            ])
            om = cv2.warpAffine(om, M, (new_w, new_h), flags=cv2.INTER_NEAREST)

        feather = _radial_feather(new_h, new_w) * (vm.astype(np.float32) / 255.0)

        angle = cfg.yaw_sign * tile.yaw_deg + cfg.yaw_axis_offset_deg
        bgr_r    = _rotate_local_float(bgr.astype(np.float32), angle)
        feather_r = _rotate_local_float(feather, angle)
        vm_r = _rotate_local_mask(vm, angle)
        gm_r = _rotate_local_mask(gm, angle)
        om_r = _rotate_local_mask(om, angle)
        wm_r = _rotate_local_mask(wm, angle)

        # Count coverage (valid tiles) and obstacle votes per canvas pixel.
        _paste_add(coverage_count, (vm_r > 0).astype(np.float32), p.cx, p.cy)
        _paste_add(occ_count, (om_r > 0).astype(np.float32), p.cx, p.cy)

        free_local = (vm_r > 0) & (gm_r == 0) & (om_r == 0) & (wm_r == 0)
        z_w = 1.0 / max(tile.z, 1e-3)

        # RGB blend.
        if use_multiband:
            _paste_multiband(mb_accum, mb_weight, bgr_r, feather_r, p.cx, p.cy, n_levels)
        elif use_seam_cut:
            _paste_seam_cut(accum_rgb, weight, bgr_r, vm_r, tile.idx, p.cx, p.cy,
                            prebuilt_seam, seam_width_px=seam_width_px)
        elif use_strip:
            s_i = strip_of.get(p.idx, 0)
            _paste_strip_seam_cut(
                accum_rgb, weight, bgr_r, vm_r,
                tile.idx, s_i, p.cx, p.cy,
                prebuilt_tile_seam, prebuilt_strip_seam,
            )
        else:
            _paste_add(accum_rgb, bgr_r * feather_r[..., None], p.cx, p.cy)
            _paste_add(weight, feather_r, p.cx, p.cy)

        # Mask votes (same for all blend modes).
        base_w = feather_r * z_w
        _paste_add(free_w, base_w * free_local.astype(np.float32), p.cx, p.cy)
        _paste_add(occ_w,  base_w * (om_r > 0).astype(np.float32), p.cx, p.cy)
        _paste_add(wall_w, base_w * (wm_r > 0).astype(np.float32), p.cx, p.cy)
        _paste_add(grid_w, base_w * (gm_r > 0).astype(np.float32), p.cx, p.cy)

        # Seam visualisation map.
        _paste_argmax_tile_id(seam, seam_top, feather_r, tile.idx, p.cx, p.cy)

    if use_multiband:
        mosaic = _reconstruct_multiband(mb_accum, mb_weight, canvas_h, canvas_w, n_levels)
    else:
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
    confident = observed & (max_p >= 0.2)

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
        if stats[k, cv2.CC_STAT_AREA] >= 200:
            cleaned[lbls == k] = 255

    # Wall: drop isolated small patches (floor markers, noise) before closing.
    wall_bin = (label == L_WALL).astype(np.uint8) * 255
    num_w, lbls_w, stats_w, _ = cv2.connectedComponentsWithStats(wall_bin, connectivity=8)
    wall_bin = np.zeros_like(wall_bin)
    for k in range(1, num_w):
        if stats_w[k, cv2.CC_STAT_AREA] >= 300:
            wall_bin[lbls_w == k] = 255
    wall_bin = cv2.morphologyEx(wall_bin, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))

    # Reassemble cleaned label.
    label_clean = label.copy()
    label_clean[label == L_OCCUPIED] = L_FREE  # reset occupied first
    label_clean[cleaned > 0] = L_OCCUPIED
    label_clean[wall_bin > 0] = L_WALL

    # Final pass: remove tiny isolated obstacle/wall clusters (floor markers, noise).
    # Real obstacles start at ~2800px; anything below 300px is noise or a floor marker.
    combined_bin = ((label_clean == L_OCCUPIED) | (label_clean == L_WALL)).astype(np.uint8) * 255
    num_f, lbls_f, stats_f, _ = cv2.connectedComponentsWithStats(combined_bin, connectivity=8)
    for k in range(1, num_f):
        if stats_f[k, cv2.CC_STAT_AREA] < 300:
            label_clean[lbls_f == k] = L_FREE

    # Mosaic-level marker pass: detect ArUco/floor-marker visual pattern in the
    # blended mosaic (highest quality signal, all tiles combined) and override to
    # free.  Catches markers that survived per-tile masking because only a minority
    # of tiles had them masked.
    mm_mosaic = marker_mask(mosaic)
    if mm_mosaic.any():
        label_clean[mm_mosaic > 0] = L_FREE

    prob_stack = np.stack([P_free, P_occ, P_wall, P_grid], axis=0).astype(np.float32)
    seam_vis = _colorize_seam(seam)
    return mosaic, label_clean, seam_vis, prob_stack, occ_count, coverage_count


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
    max_id = int(seam.max())
    for tid in range(max(16, max_id + 1)):
        out[seam == tid] = palette[tid % len(palette)]
    return out


def mask_exterior(
    label: np.ndarray,
    mosaic: np.ndarray,
    tape_pad_px: int = 30,
    wall_boundary_px: int = 150,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Mask out areas outside the arena using the tape grid as the boundary.

    Computes the convex hull of all L_GRID pixels, expands it by tape_pad_px
    (default 30px ≈ 6cm at 500px/m, just beyond the outer edge of the last
    tape strip).  Keeping this small prevents including the wall/room surfaces
    that some boundary tiles capture beyond the tape grid.  This is more robust
    than CC-based approaches because it does not leak through wall gaps.

    wall_boundary_px: L_WALL labels deeper than this many pixels inside the
    hull boundary are set to L_FREE.  Real arena netting/poles are at the
    perimeter; floor markers (ArUco etc.) that trigger wall_mask are in the
    interior.  At 500px/m, default 150px = 30cm from the boundary.

    Returns (label_masked, mosaic_masked, interior_mask bool H×W).
    """
    h, w = label.shape

    grid_pixels = label == L_GRID
    if not grid_pixels.any():
        return label.copy(), mosaic.copy(), np.ones((h, w), dtype=bool)

    ys, xs = np.where(grid_pixels)
    pts = np.column_stack([xs, ys]).astype(np.float32)
    hull = cv2.convexHull(pts)

    hull_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(hull_mask, [hull.astype(np.int32)], 255)

    if tape_pad_px > 0:
        k = np.ones((tape_pad_px * 2 + 1, tape_pad_px * 2 + 1), np.uint8)
        hull_mask = cv2.dilate(hull_mask, k)

    interior_mask = hull_mask > 0

    label_out = label.copy()
    label_out[~interior_mask] = L_UNKNOWN

    # Remove wall labels far from the arena boundary.  Real walls (netting,
    # poles) are at the perimeter; ArUco/floor markers that accidentally
    # trigger wall_mask sit in the interior.
    if wall_boundary_px > 0:
        k2 = np.ones((wall_boundary_px * 2 + 1, wall_boundary_px * 2 + 1), np.uint8)
        deep_interior = cv2.erode(hull_mask, k2) > 0
        label_out[(label_out == L_WALL) & deep_interior] = L_FREE

    mosaic_out = mosaic.copy()
    mosaic_out[~interior_mask] = 0

    return label_out, mosaic_out, interior_mask


def build_final_label(
    mosaic: np.ndarray,
    interior_mask: np.ndarray,
    label_grid_ref: np.ndarray,
    occ_count: Optional[np.ndarray] = None,
    coverage_count: Optional[np.ndarray] = None,
    occ_min_frac: float = 0.12,
    occ_close_px: int = 12,
    wall_boundary_px: int = 150,
) -> np.ndarray:
    """Build a clean occupancy label map purely from color segmentation of the mosaic.

    Replaces voting-based obstacle/wall labels with direct color detection on the
    blended mosaic (highest quality image, no per-tile perspective noise):
      - L_OCCUPIED : blue/cyan pixels (obstacle_mask) inside the interior, after
                     morphological closing to fill gaps in box faces.
      - L_WALL     : white/bright low-sat pixels (wall_mask) restricted to the
                     near-boundary zone (arena netting, not floor markers).
      - L_FREE     : interior pixels that are neither obstacle nor wall.
      - L_GRID     : kept from the voting label (tape grid, used as reference).
      - L_UNKNOWN  : outside interior_mask.
    """
    from .preprocess import obstacle_mask as _obstacle_mask, wall_mask as _wall_mask

    h, w = mosaic.shape[:2]
    label_out = np.full((h, w), L_UNKNOWN, dtype=np.uint8)
    label_out[interior_mask] = L_FREE

    # Keep tape grid from voting (reliable, distinctive orange colour).
    label_out[label_grid_ref == L_GRID] = L_GRID

    # ── Obstacle: detect on the blended mosaic using LAB b* channel.
    # b* < 128 = blue-shifted; floor is weakly blue (b* ~110-127), real boxes are
    # strongly blue/cyan (b* < 100).  This threshold cleanly separates them where
    # HSV value alone fails because the simulation floor is a dark blue-gray.
    lab_mosaic = cv2.cvtColor(mosaic, cv2.COLOR_BGR2LAB)
    om = (lab_mosaic[:, :, 2].astype(np.int16) < 110).astype(np.uint8) * 255
    om[~interior_mask] = 0

    # Parallax gate: a floor pixel truly under a box is occluded (sees blue)
    # from EVERY camera position, while the sideways "splay" of tall box faces
    # is blue only from cameras on one side.  Requiring a minimum fraction of
    # covering tiles to vote obstacle trims the splay down to the footprint.
    if occ_count is not None and coverage_count is not None and occ_min_frac > 0:
        vote_frac = occ_count / np.maximum(coverage_count, 1.0)
        om[vote_frac < occ_min_frac] = 0

    if occ_close_px > 0:
        kc = np.ones((occ_close_px * 2 + 1, occ_close_px * 2 + 1), np.uint8)
        om = cv2.morphologyEx(om, cv2.MORPH_CLOSE, kc)
    om = cv2.morphologyEx(om, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    # Fit geometric shapes to each obstacle blob: rotated rect (box) or circle.
    # Circularity = 4π·area/perimeter² → ~1.0 for circles, ~0.7 for squares.
    # If the fitted shape is >3× the blob area (bad fit from merged blobs),
    # fall back to the convex hull of the original blob.
    # Max blob area to attempt geometric fitting — blobs larger than this are
    # wall-sized structures; keep them as-is (convex hull) without rect/circle fit.
    # At 500px/m: 60000px ≈ a 49cm×49cm square (roughly 1 arena tile).
    MAX_BOX_AREA = 60_000

    occ_shapes = np.zeros_like(om)
    cnts, _ = cv2.findContours(om, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < 400:
            continue
        if area > MAX_BOX_AREA:
            # Large blob (box row): median-width rectangle along the principal
            # axis — robust against one-sided parallax wedge at arena edges.
            cv2.fillPoly(occ_shapes, [_median_width_rect(cnt, om.shape)], 255)
            continue
        perim = cv2.arcLength(cnt, True)
        circularity = (4 * np.pi * area / (perim * perim)) if perim > 0 else 0
        if circularity > 0.72:
            (cx_f, cy_f), radius = cv2.minEnclosingCircle(cnt)
            if np.pi * radius * radius <= 3.0 * area:
                cv2.circle(occ_shapes, (int(cx_f), int(cy_f)), int(radius), 255, -1)
                continue
        else:
            rect = cv2.minAreaRect(cnt)
            if rect[1][0] * rect[1][1] <= 3.0 * area:
                cv2.fillPoly(occ_shapes, [cv2.boxPoints(rect).astype(np.int32)], 255)
                continue
        cv2.fillPoly(occ_shapes, [cv2.convexHull(cnt)], 255)

    # Occupied wins over everything inside a fitted shape — tape-grid pixels
    # seen through/behind a box (parallax) must not punch free slits into the
    # obstacle on the AMR map.
    label_out[occ_shapes > 0] = L_OCCUPIED

    # ── Wall (white netting, boundary zone only) ────────────────────────────
    hull_u8 = interior_mask.astype(np.uint8) * 255
    if wall_boundary_px > 0:
        kw = np.ones((wall_boundary_px * 2 + 1, wall_boundary_px * 2 + 1), np.uint8)
        deep = cv2.erode(hull_u8, kw) > 0
        near_boundary = interior_mask & ~deep
    else:
        near_boundary = interior_mask

    wm = _wall_mask(mosaic)
    wm[~near_boundary] = 0
    wm = cv2.morphologyEx(wm, cv2.MORPH_CLOSE, np.ones((21, 21), np.uint8))
    num_w, lbls_w, stats_w, _ = cv2.connectedComponentsWithStats(wm, connectivity=8)
    for k in range(1, num_w):
        if stats_w[k, cv2.CC_STAT_AREA] >= 300:
            label_out[lbls_w == k] = L_WALL

    return label_out


def _median_width_rect(cnt: np.ndarray, shape_hw: Tuple[int, int]) -> np.ndarray:
    """Fit a rectangle of median width along the blob's principal axis.

    Robust against one-sided parallax splay: a tall box row seen only from one
    side grows a wedge toward the unseen side; the median cross-width along the
    spine ignores the bulge.  Returns the 4 corner points (int32, 4x2).
    """
    mask = np.zeros(shape_hw, dtype=np.uint8)
    cv2.drawContours(mask, [cnt], -1, 255, -1)
    ys, xs = np.nonzero(mask)
    pts = np.column_stack([xs, ys]).astype(np.float64)
    mean = pts.mean(axis=0)
    centered = pts - mean
    cov = np.cov(centered.T)
    evals, evecs = np.linalg.eigh(cov)
    axis = evecs[:, np.argmax(evals)]          # principal (long) direction
    perp = np.array([-axis[1], axis[0]])

    t = centered @ axis                         # position along the spine
    s = centered @ perp                         # offset across the spine
    nbins = max(8, int((t.max() - t.min()) / 20))
    bins = np.linspace(t.min(), t.max(), nbins + 1)
    widths, centers = [], []
    for b0, b1 in zip(bins[:-1], bins[1:]):
        sel = (t >= b0) & (t < b1)
        if sel.sum() < 10:
            continue
        s_sel = s[sel]
        lo, hi = np.percentile(s_sel, [5, 95])
        widths.append(hi - lo)
        centers.append((lo + hi) / 2.0)
    half_w = float(np.median(widths)) / 2.0
    s_mid = float(np.median(centers))

    t0, t1 = float(t.min()), float(t.max())
    corners = [
        mean + t0 * axis + (s_mid - half_w) * perp,
        mean + t1 * axis + (s_mid - half_w) * perp,
        mean + t1 * axis + (s_mid + half_w) * perp,
        mean + t0 * axis + (s_mid + half_w) * perp,
    ]
    return np.array(corners, dtype=np.int32)


def crop_to_arena(
    label: np.ndarray,
    mosaic: np.ndarray,
    arena_m: float,
    ppm: float,
) -> Tuple[np.ndarray, np.ndarray, Tuple[int, int]]:
    """Crop/pad label and mosaic to a fixed arena_m × arena_m canvas.

    Centers the window on the centroid of L_GRID pixels (tape grid = arena
    reference).  Regions outside the current canvas are filled with L_UNKNOWN /
    black so the output is always exactly (arena_px × arena_px).

    Returns (label_out, mosaic_out, (x0, y0)) where (x0, y0) is the top-left
    of the crop window in the original canvas — needed to recompute the ROS map
    world origin.
    """
    arena_px = int(round(arena_m * ppm))
    h, w = label.shape

    ys, xs = np.where(label == L_GRID)
    if len(ys) == 0:
        ys, xs = np.where(label != L_UNKNOWN)
    cx = int(round(float(xs.mean())))
    cy = int(round(float(ys.mean())))

    x0 = cx - arena_px // 2
    y0 = cy - arena_px // 2
    x1 = x0 + arena_px
    y1 = y0 + arena_px

    label_out = np.full((arena_px, arena_px), L_UNKNOWN, dtype=np.uint8)
    mosaic_out = np.zeros((arena_px, arena_px, 3), dtype=np.uint8)

    sx0, sy0 = max(0, x0), max(0, y0)
    sx1, sy1 = min(w, x1), min(h, y1)
    dx0, dy0 = sx0 - x0, sy0 - y0
    dx1, dy1 = dx0 + (sx1 - sx0), dy0 + (sy1 - sy0)

    label_out[dy0:dy1, dx0:dx1] = label[sy0:sy1, sx0:sx1]
    mosaic_out[dy0:dy1, dx0:dx1] = mosaic[sy0:sy1, sx0:sx1]

    return label_out, mosaic_out, (x0, y0)


def label_to_grid_vis(
    label: np.ndarray,
    tape_period_px: float = 265.0,
) -> np.ndarray:
    """Clean binary occupancy-grid visualization.

    Free / Unknown / Grid → white (255)
    Obstacle / Wall       → black (  0)
    Black border frame (3px) around the full map.
    """
    out = np.full(label.shape, 255, dtype=np.uint8)  # everything white by default
    out[label == L_OCCUPIED] = 0
    out[label == L_WALL]     = 0
    out_bgr = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)

    # Black border frame.
    cv2.rectangle(out_bgr, (0, 0), (out_bgr.shape[1] - 1, out_bgr.shape[0] - 1),
                  (0, 0, 0), 3)

    return out_bgr


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
