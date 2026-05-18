"""Phase C: global least-squares position solve over pairwise offsets (Guide §4.3)."""

from __future__ import annotations

from typing import Dict, List, Tuple

import numpy as np
from scipy.sparse import lil_matrix, csr_matrix
from scipy.sparse.linalg import lsqr

from .place_pose import PlacedTile
from .refine_window import PairResult


def solve_positions(
    placed: List[PlacedTile],
    pairs: List[PairResult],
    anchor_idx: int = 0,
) -> Dict[int, Tuple[float, float]]:
    """Returns {idx: (cx, cy)} refined canvas centers.

    Variables: (tx_i, ty_i) per tile.
    Per accepted pair (i, j):  tx_j - tx_i = dx_refined,  ty_j - ty_i = dy_refined.
    Prior rows: soft-anchor every tile to its Phase A position so tiles not
    covered by any accepted pair stay where Phase A placed them (instead of
    collapsing to the mathematical origin in the underdetermined lsqr solve).
    Anchor: tx_anchor, ty_anchor fixed hard via a very high-weight row.
    """
    ids = sorted({p.idx for p in placed})
    idx_to_var = {idx: k for k, idx in enumerate(ids)}
    N = len(ids)
    by_idx_map = {p.idx: p for p in placed}

    accepted = [p for p in pairs if p.accepted]
    n_pairs = len(accepted)
    n_priors = N  # one soft-prior row per tile
    rows = n_pairs + n_priors + 1  # +1 heavy anchor

    A_x = lil_matrix((rows, N), dtype=np.float64)
    A_y = lil_matrix((rows, N), dtype=np.float64)
    bx = np.zeros(rows, dtype=np.float64)
    by = np.zeros(rows, dtype=np.float64)

    # Pair-offset constraints (weight 1).
    for r, pr in enumerate(accepted):
        ki = idx_to_var[pr.i]; kj = idx_to_var[pr.j]
        A_x[r, kj] = 1.0; A_x[r, ki] = -1.0
        bx[r] = pr.dx_refined
        A_y[r, kj] = 1.0; A_y[r, ki] = -1.0
        by[r] = pr.dy_refined

    # Soft prior: each tile attracted to its Phase A center (weight 1).
    # Tiles with no accepted pairs are held entirely by their prior.
    W_PRIOR = 1.0
    for k, idx in enumerate(ids):
        r = n_pairs + k
        p = by_idx_map[idx]
        A_x[r, k] = W_PRIOR; bx[r] = W_PRIOR * p.cx
        A_y[r, k] = W_PRIOR; by[r] = W_PRIOR * p.cy

    # Heavy anchor: fix one tile completely so the solve has a global reference.
    if anchor_idx not in idx_to_var:
        anchor_idx = ids[0]
    ka = idx_to_var[anchor_idx]
    W_ANCHOR = 1e4
    r_anc = n_pairs + n_priors
    A_x[r_anc, ka] = W_ANCHOR; bx[r_anc] = W_ANCHOR * by_idx_map[anchor_idx].cx
    A_y[r_anc, ka] = W_ANCHOR; by[r_anc] = W_ANCHOR * by_idx_map[anchor_idx].cy

    tx = lsqr(csr_matrix(A_x), bx)[0]
    ty = lsqr(csr_matrix(A_y), by)[0]

    return {idx: (float(tx[idx_to_var[idx]]), float(ty[idx_to_var[idx]])) for idx in ids}


def apply_positions(placed: List[PlacedTile], positions: Dict[int, Tuple[float, float]]) -> None:
    for p in placed:
        if p.idx in positions:
            p.cx, p.cy = positions[p.idx]


def summarize_residuals(pairs: List[PairResult], positions: Dict[int, Tuple[float, float]]) -> str:
    accepted = [p for p in pairs if p.accepted]
    if not accepted:
        return "no accepted pairs"
    res = []
    for pr in accepted:
        if pr.i not in positions or pr.j not in positions:
            continue
        ix, iy = positions[pr.i]
        jx, jy = positions[pr.j]
        rx = (jx - ix) - pr.dx_refined
        ry = (jy - iy) - pr.dy_refined
        res.append((rx, ry))
    arr = np.array(res)
    return (
        f"pairs={len(accepted)} rejected={len(pairs) - len(accepted)} "
        f"mean|res|=({float(np.mean(np.abs(arr[:, 0]))):.2f}, {float(np.mean(np.abs(arr[:, 1]))):.2f})px "
        f"max|res|=({float(np.max(np.abs(arr[:, 0]))):.2f}, {float(np.max(np.abs(arr[:, 1]))):.2f})px"
    )
