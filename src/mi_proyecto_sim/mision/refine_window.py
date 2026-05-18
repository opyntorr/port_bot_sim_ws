"""Phase B: sliding-window pairwise refinement on edge images (Guide §4.2).

Uses cv2.matchTemplate on the local overlap region (no full-canvas warps), so
pairs refine in seconds rather than minutes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import cv2
import numpy as np

from .place_pose import PlacedTile


def _compute_adjacency(placed: List[PlacedTile]) -> List[Tuple[int, int]]:
    """Build adjacent pairs from bounding-box overlap.

    Two tiles are adjacent when their bboxes overlap AND the overlap in at least
    one axis exceeds 50 % of the footprint.  This selects direct grid neighbours
    (row or column) while excluding diagonal pairs whose overlap fraction is
    typically < 50 % in both axes.  Works for any grid density or waypoint count.
    """
    if not placed:
        return []
    pairs: List[Tuple[int, int]] = []
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            pi, pj = placed[i], placed[j]
            fp = (pi.footprint_px + pj.footprint_px) / 2.0
            # Use square footprint (not the rotated warped bbox) so diagonal
            # pairs don't sneak in due to rotation-expanded bounding boxes.
            overlap_x = min(pi.cx + fp / 2, pj.cx + fp / 2) - max(pi.cx - fp / 2, pj.cx - fp / 2)
            overlap_y = min(pi.cy + fp / 2, pj.cy + fp / 2) - max(pi.cy - fp / 2, pj.cy - fp / 2)
            if overlap_x <= 0 or overlap_y <= 0:
                continue
            if max(overlap_x / fp, overlap_y / fp) > 0.5:
                pairs.append((pi.idx, pj.idx))
    return pairs


@dataclass
class RefineConfig:
    search_radius_px: int = 60
    min_score: float = 0.08
    # Lower bound on template area, in pixels, below which a pair is skipped.
    min_template_area: int = 5000


@dataclass
class PairResult:
    i: int
    j: int
    dx_prior: float
    dy_prior: float
    dx_refined: float
    dy_refined: float
    score: float
    accepted: bool


def _local_edge(placed: PlacedTile, edge_func) -> np.ndarray:
    """Image (float32 in [0,1]) for this tile in *local* (warped-tile) coords.

    Only the warp-rotation border (pure-black BORDER_CONSTANT padding) is zeroed
    out.  warped_valid is NOT used here because it also masks dark floor pixels,
    which would erase most of the overlap content on an arena with a black floor.
    """
    e = edge_func(placed.warped_bgr).astype(np.float32) / 255.0
    # max-channel > 1 → not the pure-black rotation border
    border_mask = (placed.warped_bgr.max(axis=2) > 1).astype(np.float32)
    e *= border_mask
    return e


def _placed_bbox(p: PlacedTile) -> Tuple[int, int, int, int]:
    """Return (x0, y0, x1, y1) of this placed tile in canvas coords."""
    h, w = p.warped_bgr.shape[:2]
    x0 = int(round(p.cx - w / 2.0))
    y0 = int(round(p.cy - h / 2.0))
    return x0, y0, x0 + w, y0 + h


def _crop_overlap_with_margin(
    edge: np.ndarray, ox0: int, oy0: int, ox1: int, oy1: int,
    bbox: Tuple[int, int, int, int], margin: int,
) -> Tuple[np.ndarray, int, int]:
    """Crop `edge` (in local coords of bbox) to the rectangle (ox0,oy0,ox1,oy1)
    expressed in canvas coords, expanded by `margin` on each side, clipped to bbox.

    Returns (crop, canvas_x0_of_crop, canvas_y0_of_crop).
    """
    bx0, by0, bx1, by1 = bbox
    cx0 = max(bx0, ox0 - margin)
    cy0 = max(by0, oy0 - margin)
    cx1 = min(bx1, ox1 + margin)
    cy1 = min(by1, oy1 + margin)
    if cx1 <= cx0 or cy1 <= cy0:
        return np.empty((0, 0), dtype=np.float32), cx0, cy0
    # Convert canvas coords to local coords inside `edge`.
    lx0 = cx0 - bx0; ly0 = cy0 - by0
    lx1 = cx1 - bx0; ly1 = cy1 - by0
    return edge[ly0:ly1, lx0:lx1].copy(), cx0, cy0


def refine_pair(
    pi: PlacedTile, pj: PlacedTile,
    edge_func, cfg: RefineConfig,
) -> PairResult:
    """Refine relative offset between tiles i and j using phase correlation.

    Phase correlation works in the Fourier domain and places its peak at the
    true sub-pixel shift regardless of the grid period.  Unlike matchTemplate
    (spatial domain), it cannot lock onto a wrong grid-period multiple.

    phaseCorrelate(A, B) returns shift s such that B(x) = A(x - s), i.e. the
    shift FROM A TO B.  To correct tile j's position we subtract the returned
    shift from the prior offset.
    """
    bi = _placed_bbox(pi)
    bj = _placed_bbox(pj)
    bxi0, byi0, bxi1, byi1 = bi
    bxj0, byj0, bxj1, byj1 = bj

    # Overlap rect in canvas coords (under prior placement).
    ox0 = max(bxi0, bxj0); oy0 = max(byi0, byj0)
    ox1 = min(bxi1, bxj1); oy1 = min(byi1, byj1)
    if ox1 <= ox0 or oy1 <= oy0:
        return PairResult(pi.idx, pj.idx, pj.cx - pi.cx, pj.cy - pi.cy,
                          pj.cx - pi.cx, pj.cy - pi.cy, score=-1.0, accepted=False)

    ow, oh = ox1 - ox0, oy1 - oy0
    if oh < 16 or ow < 16 or oh * ow < cfg.min_template_area:
        return PairResult(pi.idx, pj.idx, pj.cx - pi.cx, pj.cy - pi.cy,
                          pj.cx - pi.cx, pj.cy - pi.cy, score=0.0, accepted=False)

    ei = _local_edge(pi, edge_func)
    ej = _local_edge(pj, edge_func)

    # Extract the overlap region in each tile's local coords.
    pi_y0, pi_x0 = oy0 - byi0, ox0 - bxi0
    pj_y0, pj_x0 = oy0 - byj0, ox0 - bxj0
    patch_i = ei[pi_y0:pi_y0 + oh, pi_x0:pi_x0 + ow].copy()
    patch_j = ej[pj_y0:pj_y0 + oh, pj_x0:pj_x0 + ow].copy()

    # Guard against bbox clipping edge cases.
    h = min(patch_i.shape[0], patch_j.shape[0])
    w = min(patch_i.shape[1], patch_j.shape[1])
    if h < 16 or w < 16 or h * w < cfg.min_template_area:
        return PairResult(pi.idx, pj.idx, pj.cx - pi.cx, pj.cy - pi.cy,
                          pj.cx - pi.cx, pj.cy - pi.cy, score=0.0, accepted=False)

    patch_i = patch_i[:h, :w].astype(np.float32)
    patch_j = patch_j[:h, :w].astype(np.float32)

    # Hanning window reduces spectral leakage from patch edges.
    win = cv2.createHanningWindow((w, h), cv2.CV_32F)
    patch_i *= win
    patch_j *= win

    # Phase correlation: robust to periodic floor patterns.
    # Returns (shift_x, shift_y) where shift is FROM patch_i TO patch_j,
    # so the correction to tile j's canvas position is the negative of shift.
    try:
        (sx, sy), response = cv2.phaseCorrelate(patch_i, patch_j)
    except cv2.error:
        return PairResult(pi.idx, pj.idx, pj.cx - pi.cx, pj.cy - pi.cy,
                          pj.cx - pi.cx, pj.cy - pi.cy, score=0.0, accepted=False)

    R = cfg.search_radius_px
    sx = float(np.clip(sx, -R, R))
    sy = float(np.clip(sy, -R, R))

    dx_prior = pj.cx - pi.cx
    dy_prior = pj.cy - pi.cy
    dx_ref = dx_prior - sx
    dy_ref = dy_prior - sy

    score = float(response)
    # Reject if either axis hit the clamp boundary: the true correlation peak
    # is outside the search window, meaning this is an aliased or noise-driven result.
    clamped = abs(sx) >= R * 0.95 or abs(sy) >= R * 0.95
    return PairResult(pi.idx, pj.idx, dx_prior, dy_prior, dx_ref, dy_ref,
                      score=score, accepted=score >= cfg.min_score and not clamped)


def refine_all(
    spec_h: int, spec_w: int,
    placed: List[PlacedTile],
    edge_func,
    cfg: RefineConfig | None = None,
) -> List[PairResult]:
    """spec_h/spec_w kept in signature for backward compatibility; unused now."""
    del spec_h, spec_w
    cfg = cfg or RefineConfig()
    by_idx: Dict[int, PlacedTile] = {p.idx: p for p in placed}
    adjacency = _compute_adjacency(placed)
    results: List[PairResult] = []
    for i, j in adjacency:
        if i not in by_idx or j not in by_idx:
            continue
        results.append(refine_pair(by_idx[i], by_idx[j], edge_func, cfg))
    return results


# -----------------------------------------------------------------------------
# Lightweight scoring helpers (used by debug_yaw.py to rank yaw conventions).
# -----------------------------------------------------------------------------


def pair_overlap_score(pi: PlacedTile, pj: PlacedTile, edge_func) -> float:
    """NCC of the joint-valid edge overlap region under current placement."""
    ei = _local_edge(pi, edge_func)
    ej = _local_edge(pj, edge_func)
    bi = _placed_bbox(pi); bj = _placed_bbox(pj)
    ox0 = max(bi[0], bj[0]); oy0 = max(bi[1], bj[1])
    ox1 = min(bi[2], bj[2]); oy1 = min(bi[3], bj[3])
    if ox1 <= ox0 or oy1 <= oy0:
        return -1.0
    li_x0 = ox0 - bi[0]; li_y0 = oy0 - bi[1]
    li_x1 = ox1 - bi[0]; li_y1 = oy1 - bi[1]
    lj_x0 = ox0 - bj[0]; lj_y0 = oy0 - bj[1]
    lj_x1 = ox1 - bj[0]; lj_y1 = oy1 - bj[1]
    a = ei[li_y0:li_y1, li_x0:li_x1]
    b = ej[lj_y0:lj_y1, lj_x0:lj_x1]
    mask = (a > 1e-6) & (b > 1e-6)
    n = int(mask.sum())
    if n < 500:
        return -1.0
    a1 = a[mask]; b1 = b[mask]
    a1 = a1 - a1.mean(); b1 = b1 - b1.mean()
    denom = (np.linalg.norm(a1) * np.linalg.norm(b1)) + 1e-9
    return float((a1 * b1).sum() / denom)


def mean_adjacency_score(placed: List[PlacedTile], edge_func) -> float:
    by_idx = {p.idx: p for p in placed}
    scores = []
    for i, j in ADJACENCY:
        if i not in by_idx or j not in by_idx:
            continue
        s = pair_overlap_score(by_idx[i], by_idx[j], edge_func)
        if s > -0.5:
            scores.append(s)
    return float(np.mean(scores)) if scores else 0.0
