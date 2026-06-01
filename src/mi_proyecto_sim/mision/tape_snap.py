"""Phase A+: snap tile positions to the global tape grid (Guide extension).

After Phase A (pose-prior placement + image-yaw rotation), this module detects
where each tile's tape lines actually land in canvas coordinates and shifts each
tile so all tape lines align to a single global 90° grid.

Two phase-detection methods are available (set via method= parameter):

  'projection' — original: smooth the 1-D tape projection and detect peaks.
      Works well when the tile's rotation is small (< ~6°), but smears peaks
      when the tape lines are significantly tilted in the warped image.

  'derotate'   — de-rotate the tape mask by -rotation_deg before projecting,
      making tape lines axis-aligned regardless of the tile's heading error.
      Avoids smearing but introduces minor warpAffine interpolation artefacts.

  'hough'      — detect tape lines via HoughLines on Canny edges of the tape mask,
      then compute the Y/X-intercept of each line at the tile centre.  Natively
      handles any line angle without resampling; naturally robust to isolated
      blobs that confuse a 1-D projection.

  'auto'       — (default) uses 'derotate' when |rotation_deg| > 3°, else
      'projection'.  This covers the low-yaw tiles the same as before while
      fixing the high-yaw tiles that were previously skipped.

Usage:
    from .tape_snap import snap_to_tape_grid
    n_snapped = snap_to_tape_grid(placed, ppm=500.0)
    n_snapped = snap_to_tape_grid(placed, ppm=500.0, method='hough')
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np

from .place_pose import PlacedTile

_HSV_LO = np.array([8,  80,  60], dtype=np.uint8)
_HSV_HI = np.array([28, 255, 255], dtype=np.uint8)


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _tape_mask(bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, _HSV_LO, _HSV_HI)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)


def _find_tape_peaks(proj: np.ndarray, period_px: float) -> List[float]:
    """Find tape line centres in a 1-D projection via Gaussian smoothing + peak find."""
    n = len(proj)
    if n < int(period_px * 0.5):
        return []

    sigma = max(2.0, period_px / 8.0)
    ksize = int(sigma * 6) | 1
    smooth = cv2.GaussianBlur(
        proj.astype(np.float32).reshape(1, -1),
        (ksize, 1), sigma,
    ).ravel()

    h_max = float(smooth.max())
    if h_max < 1.0:
        return []
    threshold = 0.30 * h_max
    min_dist = int(period_px * 0.5)

    peaks = []
    i = 1
    while i < n - 1:
        if smooth[i] >= threshold and smooth[i] >= smooth[i - 1] and smooth[i] >= smooth[i + 1]:
            denom = smooth[i - 1] - 2.0 * smooth[i] + smooth[i + 1]
            offset = 0.5 * (smooth[i - 1] - smooth[i + 1]) / denom if abs(denom) > 1e-6 else 0.0
            peak_pos = float(i) + offset
            if not peaks or (peak_pos - peaks[-1]) >= min_dist:
                peaks.append(peak_pos)
            elif smooth[i] > smooth[int(round(peaks[-1]))]:
                peaks[-1] = peak_pos
            i += min_dist
        else:
            i += 1
    return peaks


def _phase_from_peaks(peaks: List[float], period_px: float) -> Optional[float]:
    """Circular mean of (peak % T) estimates. Returns None if no peaks."""
    if not peaks:
        return None
    estimates = [p % period_px for p in peaks]
    if len(estimates) == 1:
        return float(estimates[0])
    angles = [e / period_px * 2.0 * np.pi for e in estimates]
    s = float(np.mean(np.sin(angles)))
    c = float(np.mean(np.cos(angles)))
    return (float(np.arctan2(s, c)) / (2.0 * np.pi) * period_px) % period_px


def _circular_mean(values: List[float], period: float) -> float:
    angles = [v / period * 2.0 * np.pi for v in values]
    s = float(np.mean(np.sin(angles)))
    c = float(np.mean(np.cos(angles)))
    return (float(np.arctan2(s, c)) / (2.0 * np.pi) * period) % period


def _robust_ref_phase(phases: List[float], period: float) -> Optional[float]:
    """Histogram-mode robust reference: finds the densest bin, returns circular mean
    of inliers within ±2 bins.  Resistant to a minority of wildly wrong estimates."""
    if not phases:
        return None
    if len(phases) == 1:
        return float(phases[0])
    n_bins = max(8, int(period / 15.0))
    hist = np.zeros(n_bins, dtype=float)
    for v in phases:
        b = int(v / period * n_bins) % n_bins
        hist[b] += 1.0
    hist_s = hist + np.roll(hist, 1) + np.roll(hist, -1)
    peak_bin = int(np.argmax(hist_s))
    inliers = []
    for v in phases:
        b = int(v / period * n_bins) % n_bins
        dist = min(abs(b - peak_bin), n_bins - abs(b - peak_bin))
        if dist <= 2:
            inliers.append(v)
    return _circular_mean(inliers if inliers else phases, period)


# ---------------------------------------------------------------------------
# Period estimation (common to all methods)
# ---------------------------------------------------------------------------

def _estimate_period(placed: List[PlacedTile], ppm: float) -> float:
    """Estimate tape period from inter-peak spacings across all tiles."""
    rough_periods = []
    for p in placed[:10]:
        m = _tape_mask(p.warped_bgr)
        if m.mean() < 2.0:
            continue
        for axis in (0, 1):
            proj = m.sum(axis=axis).astype(np.float64)
            proj -= proj.mean()
            n = len(proj)
            fft = np.abs(np.fft.rfft(proj))
            lo = max(1, int(n / 400))
            hi = min(len(fft) - 1, int(n / 80))
            if lo >= hi:
                continue
            k = int(np.argmax(fft[lo:hi + 1])) + lo
            rough_periods.append(n / k)

    rough_T = float(np.median(rough_periods)) if rough_periods else ppm * 0.5

    spacings: List[float] = []
    for p in placed:
        m = _tape_mask(p.warped_bgr)
        if m.mean() < 2.0:
            continue
        for axis in (0, 1):
            proj = m.sum(axis=axis).astype(np.float64)
            peaks = _find_tape_peaks(proj, rough_T)
            for i in range(1, len(peaks)):
                spacings.append(peaks[i] - peaks[i - 1])

    if not spacings:
        return rough_T
    good = [s for s in spacings if abs(s - rough_T) < 0.3 * rough_T]
    return float(np.median(good)) if good else rough_T


# ---------------------------------------------------------------------------
# Method A: de-rotate mask before 1-D projection
# ---------------------------------------------------------------------------

def _tape_phase_derotate(
    mask: np.ndarray,
    rotation_deg: float,
    period_px: float,
) -> Tuple[Optional[float], Optional[float]]:
    """Phase detection by fully undoing the warp rotation before projecting.

    OpenCV's getRotationMatrix2D with positive angle makes originally-horizontal
    content CW-tilted in the output.  When prepare_tile applied +rotation_deg to
    correct a drone-heading offset of -rotation_deg, it actually doubled the CW
    tilt (original tilt α + added tilt α = 2α in warped_bgr).

    To make tape lines axis-aligned for a clean 1-D projection we therefore
    apply -2*rotation_deg (not -rotation_deg, which only halves the residual).
    """
    h, w = mask.shape
    M = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), -2.0 * rotation_deg, 1.0)
    derotated = cv2.warpAffine(mask, M, (w, h), flags=cv2.INTER_NEAREST)
    peaks_y = _find_tape_peaks(derotated.sum(axis=1).astype(np.float64), period_px)
    peaks_x = _find_tape_peaks(derotated.sum(axis=0).astype(np.float64), period_px)
    return _phase_from_peaks(peaks_y, period_px), _phase_from_peaks(peaks_x, period_px)


# ---------------------------------------------------------------------------
# Method B: HoughLines Y/X intercept at tile centre
# ---------------------------------------------------------------------------

def _cluster_values(values: List[float], min_dist: float) -> List[float]:
    """Merge values within min_dist of each other; return cluster means."""
    if not values:
        return []
    sv = sorted(values)
    clusters: List[List[float]] = [[sv[0]]]
    for v in sv[1:]:
        if v - clusters[-1][-1] < min_dist:
            clusters[-1].append(v)
        else:
            clusters.append([v])
    return [float(np.mean(c)) for c in clusters]


def _tape_phase_hough(
    bgr: np.ndarray,
    rotation_deg: float,  # kept for API compatibility, no longer used for angle filter
    period_px: float,
) -> Tuple[Optional[float], Optional[float]]:
    """Phase detection via HoughLines intercept at tile centre (angle-agnostic).

    Finds the dominant tape angle directly from HoughLines votes rather than
    assuming it from rotation_deg.  This is correct regardless of whether the
    tile's warp rotation corrected or doubled the original tape tilt.

    For each detected line in the horizontal family (theta in pi/4..3pi/4),
    computes the Y-intercept at x=w/2.  That intercept equals the true canvas-Y
    position of the tape line at the tile centre — exact, no cosine-compression
    artefact.  Same for the vertical family (X-intercept at y=h/2).
    """
    h, w = bgr.shape[:2]

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    tape = cv2.inRange(hsv, _HSV_LO, _HSV_HI)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    tape = cv2.morphologyEx(tape, cv2.MORPH_CLOSE, k)

    if float(tape.mean()) < 2.0:
        return None, None

    edges = cv2.Canny(tape, 30, 90)
    lines = cv2.HoughLines(edges, rho=1, theta=np.pi / 720, threshold=80)
    if lines is None:
        return None, None

    rhos   = lines[:, 0, 0]
    thetas = lines[:, 0, 1]

    tol = np.radians(10.0)

    # --- Horizontal tape family: theta in (pi/4, 3pi/4) ---
    horiz = (thetas > np.pi / 4) & (thetas < 3 * np.pi / 4)
    y_intercepts: List[float] = []
    if horiz.sum() >= 2:
        # Find dominant angle from the data; use a tight window around it.
        theta_h = float(np.median(thetas[horiz]))
        close_h = horiz & (np.abs(thetas - theta_h) < tol)
        for rho, theta in zip(rhos[close_h], thetas[close_h]):
            sin_t = np.sin(theta)
            if abs(sin_t) < 0.1:
                continue
            y_ic = (rho - (w / 2.0) * np.cos(theta)) / sin_t
            if 0 <= y_ic < h:
                y_intercepts.append(float(y_ic))

    # --- Vertical tape family: theta outside (pi/4, 3pi/4) ---
    vert = (thetas <= np.pi / 4) | (thetas >= 3 * np.pi / 4)
    x_intercepts: List[float] = []
    if vert.sum() >= 2:
        theta_v = float(np.median(thetas[vert]))
        close_v = vert & (np.abs(thetas - theta_v) < tol)
        for rho, theta in zip(rhos[close_v], thetas[close_v]):
            cos_t = np.cos(theta)
            if abs(cos_t) < 0.1:
                continue
            x_ic = (rho - (h / 2.0) * np.sin(theta)) / cos_t
            if 0 <= x_ic < w:
                x_intercepts.append(float(x_ic))

    min_cluster_dist = period_px * 0.3
    y_clustered = _cluster_values(y_intercepts, min_cluster_dist)
    x_clustered = _cluster_values(x_intercepts, min_cluster_dist)

    phase_y = _phase_from_peaks(y_clustered, period_px) if y_clustered else None
    phase_x = _phase_from_peaks(x_clustered, period_px) if x_clustered else None
    return phase_y, phase_x


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def snap_to_tape_grid(
    placed: List[PlacedTile],
    ppm: float = 500.0,
    tape_period_m: Optional[float] = None,
    max_correction_frac: float = 0.47,
    method: str = "auto",
    debug: bool = False,
) -> int:
    """Shift each tile's canvas centre so its tape lines align to a global grid.

    method: 'projection' | 'derotate' | 'hough' | 'auto'
      'auto' uses 'derotate' when |rotation_deg| > 3°, else 'projection'.

    Returns number of tiles that were shifted.
    """
    T = (_estimate_period(placed, ppm) if tape_period_m is None
         else tape_period_m * ppm)
    max_corr = max_correction_frac * T

    if debug:
        print(f"[tape_snap] T={T:.1f}px ({T/ppm*100:.1f}cm), "
              f"max_corr=±{max_corr:.0f}px, method={method!r}")

    canvas_phases_y: List[Optional[float]] = []
    canvas_phases_x: List[Optional[float]] = []

    for p in placed:
        h, w = p.warped_bgr.shape[:2]
        m = _tape_mask(p.warped_bgr)

        # Choose detection method for this tile.
        if method == "auto":
            tile_method = "hough" if abs(p.rotation_deg) > 3.0 else "projection"
        else:
            tile_method = method

        if tile_method == "derotate":
            phase_y, phase_x = _tape_phase_derotate(m, p.rotation_deg, T)
        elif tile_method == "hough":
            phase_y, phase_x = _tape_phase_hough(p.warped_bgr, p.rotation_deg, T)
        else:  # 'projection'
            phase_y = _phase_from_peaks(
                _find_tape_peaks(m.sum(axis=1).astype(np.float64), T), T)
            phase_x = _phase_from_peaks(
                _find_tape_peaks(m.sum(axis=0).astype(np.float64), T), T)

        cy_canvas = (p.cy - h / 2.0 + phase_y) % T if phase_y is not None else None
        cx_canvas = (p.cx - w / 2.0 + phase_x) % T if phase_x is not None else None

        canvas_phases_y.append(cy_canvas)
        canvas_phases_x.append(cx_canvas)

    valid_y = [v for v in canvas_phases_y if v is not None]
    valid_x = [v for v in canvas_phases_x if v is not None]

    if not valid_y and not valid_x:
        print("[tape_snap] WARN: no tape detected in any tile — skipping snap")
        return 0

    ref_y = _robust_ref_phase(valid_y, T) if valid_y else None
    ref_x = _robust_ref_phase(valid_x, T) if valid_x else None

    # Pass 1: compute wrapped deltas (None = phase missing or exceeds max_corr).
    def _wrap(delta: float) -> float:
        if delta > T / 2:
            delta -= T
        if delta < -T / 2:
            delta += T
        return delta

    raw_dy: List[Optional[float]] = []
    raw_dx: List[Optional[float]] = []
    for cy_phase, cx_phase in zip(canvas_phases_y, canvas_phases_x):
        dy: Optional[float] = None
        if ref_y is not None and cy_phase is not None:
            d = _wrap(cy_phase - ref_y)
            dy = d if abs(d) <= max_corr else None
        raw_dy.append(dy)

        dx: Optional[float] = None
        if ref_x is not None and cx_phase is not None:
            d = _wrap(cx_phase - ref_x)
            dx = d if abs(d) <= max_corr else None
        raw_dx.append(dx)

    # Pass 2: neighborhood consistency filter.
    # If a tile's correction disagrees with its local neighbors' median by more
    # than T/3, the phase detector likely locked to the wrong tape line (aliasing).
    # Replace the aliased correction with the neighborhood median so the tile still
    # snaps to the tape grid (at the consensus position) rather than staying at its
    # raw OptiTrack location, which can overlap with neighbours and duplicate objects.
    def _consistency_filter(
        deltas: List[Optional[float]], radius: int = 2, max_dev_frac: float = 0.33,
    ) -> Tuple[List[Optional[float]], List[bool]]:
        """Returns (filtered_deltas, was_replaced) — aliased entries replaced with median."""
        n = len(deltas)
        replaced = [False] * n
        if n < 5:
            return list(deltas), replaced
        max_dev = T * max_dev_frac
        result = list(deltas)
        for i in range(n):
            if result[i] is None:
                continue
            nbr = [deltas[j] for j in range(max(0, i - radius), min(n, i + radius + 1))
                   if j != i and deltas[j] is not None]
            if len(nbr) < 2:
                continue
            med = float(np.median(nbr))
            if abs(result[i] - med) > max_dev:
                result[i] = med   # snap to consensus instead of wrong tape line
                replaced[i] = True
        return result, replaced

    filt_dy, replaced_dy = _consistency_filter(raw_dy)
    filt_dx, replaced_dx = _consistency_filter(raw_dx)

    # Pass 3: apply filtered deltas.
    # Also store raw delta for tiles that exceeded max_corr (for debug SKIP label).
    over_corr_dy: List[Optional[float]] = []
    over_corr_dx: List[Optional[float]] = []
    for cy_phase, cx_phase in zip(canvas_phases_y, canvas_phases_x):
        oy: Optional[float] = None
        if ref_y is not None and cy_phase is not None:
            d = _wrap(cy_phase - ref_y)
            if abs(d) > max_corr:
                oy = d
        over_corr_dy.append(oy)
        ox: Optional[float] = None
        if ref_x is not None and cx_phase is not None:
            d = _wrap(cx_phase - ref_x)
            if abs(d) > max_corr:
                ox = d
        over_corr_dx.append(ox)

    n_snapped = 0
    n_skipped = 0
    n_alias = 0
    for i, (p, raw_y, raw_x, dy, dx) in enumerate(
        zip(placed, raw_dy, raw_dx, filt_dy, filt_dx)
    ):
        shifted = False

        if dy is not None:
            tag = f"ALIAS_FIX({raw_y:+.0f}→{dy:+.1f}px)" if replaced_dy[i] else f"{dy:+.1f}px"
            if replaced_dy[i]:
                n_alias += 1
            if debug:
                print(f"  wp_{p.idx:02d} dy={tag}  dx=", end="")
            p.cy -= dy
            shifted = True
        else:
            oc = over_corr_dy[i]
            if oc is not None:
                if debug:
                    print(f"  wp_{p.idx:02d} dy=SKIP({oc:+.0f}px)  dx=", end="")
                n_skipped += 1
            elif debug:
                print(f"  wp_{p.idx:02d} dy=n/a  dx=", end="")

        if dx is not None:
            tag = f"ALIAS_FIX({raw_x:+.0f}→{dx:+.1f}px)" if replaced_dx[i] else f"{dx:+.1f}px"
            if debug:
                print(tag)
            p.cx -= dx
            shifted = True
        elif over_corr_dx[i] is not None:
            if debug:
                print(f"SKIP({over_corr_dx[i]:+.0f}px)")
        else:
            if debug:
                print("n/a")

        if shifted:
            n_snapped += 1

    ry = f"{ref_y:.1f}" if ref_y is not None else "n/a"
    rx = f"{ref_x:.1f}" if ref_x is not None else "n/a"
    print(f"[tape_snap] snapped {n_snapped}/{len(placed)} tiles, "
          f"skipped {n_skipped} axis-corrections > {max_corr:.0f}px, "
          f"alias-fixed {n_alias} "
          f"(ref_y={ry}px, ref_x={rx}px)")

    return n_snapped
