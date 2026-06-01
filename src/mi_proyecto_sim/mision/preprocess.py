"""Per-tile preprocessing: masks, CLAHE, edge image (Guide §3)."""

from __future__ import annotations

import cv2
import numpy as np


def valid_mask(img: np.ndarray, dark_thresh: int = 15, border_px: int = 10) -> np.ndarray:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    m = (gray > dark_thresh).astype(np.uint8) * 255
    m[:border_px, :] = 0
    m[-border_px:, :] = 0
    m[:, :border_px] = 0
    m[:, -border_px:] = 0
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, k)


def clahe_lab(bgr: np.ndarray) -> np.ndarray:
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    L, A, B = cv2.split(lab)
    L = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(L)
    return cv2.cvtColor(cv2.merge([L, A, B]), cv2.COLOR_LAB2BGR)


def grid_mask(bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (8, 80, 60), (28, 255, 255))
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)


def obstacle_mask(bgr: np.ndarray) -> np.ndarray:
    """Detect blue/cyan boxes; fill each blob's convex hull for solid shapes."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (85, 60, 60), (150, 255, 255))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled = np.zeros_like(m)
    for cnt in cnts:
        if cv2.contourArea(cnt) > 3000:
            cv2.fillPoly(filled, [cv2.convexHull(cnt)], 255)
    return filled


def marker_mask(orig_bgr: np.ndarray, out_hw: "tuple[int,int] | None" = None) -> np.ndarray:
    """Detect bright square floor markers (ArUco, QR, etc.) by visual appearance.

    Works on the full-resolution tile image so the pattern details are intact.
    Criteria: bright (mean >170), roughly square (aspect >0.65), small enough to
    be a floor marker (<150px side), high internal contrast (std >30 inside),
    dark outer border.  Returns a dilated filled mask at out_hw if given.
    """
    h, w = orig_bgr.shape[:2]
    out = np.zeros((h, w), dtype=np.uint8)
    gray = cv2.cvtColor(orig_bgr, cv2.COLOR_BGR2GRAY)

    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY, 31, -5)
    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < 400 or area > 12000:
            continue
        x, y, cw, ch = cv2.boundingRect(cnt)
        if cw > 150 or ch > 150:
            continue
        if min(cw, ch) / max(cw, ch) < 0.65:
            continue
        m = np.zeros_like(gray)
        cv2.drawContours(m, [cnt], -1, 255, -1)
        if float(gray[m > 0].mean()) < 170:
            continue
        # High internal contrast (black/white cells inside the marker).
        ix, iy = x + cw // 5, y + ch // 5
        inner = gray[iy: iy + ch * 3 // 5, ix: ix + cw * 3 // 5]
        if inner.size == 0 or float(inner.std()) < 30:
            continue
        # Dark outer border (ArUco has a black frame around it).
        bx0, by0 = max(0, x - 5), max(0, y - 5)
        bx1, by1 = min(w, x + cw + 5), min(h, y + ch + 5)
        border = gray[by0:by1, bx0:bx1].copy()
        omask = np.ones_like(border) * 255
        omask[y - by0: y - by0 + ch, x - bx0: x - bx0 + cw] = 0
        if omask.any() and float(border[omask > 0].mean()) > 120:
            continue
        cv2.fillPoly(out, [cnt], 255)

    if out.any():
        out = cv2.dilate(out, np.ones((15, 15), np.uint8))
    if out_hw is not None and (h, w) != (out_hw[0], out_hw[1]):
        out = cv2.resize(out, (out_hw[1], out_hw[0]), interpolation=cv2.INTER_NEAREST)
    return out


def wall_mask(bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # High value, low saturation = white netting / poles.
    m = cv2.inRange(hsv, (0, 0, 180), (179, 60, 255))
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)


def edge_image(bgr: np.ndarray, blur_sigma: float = 2.0) -> np.ndarray:
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    if blur_sigma > 0:
        ksize = max(3, int(2 * round(2 * blur_sigma) + 1))
        edges = cv2.GaussianBlur(edges, (ksize, ksize), blur_sigma)
    return edges


def estimate_tile_yaw(bgr: np.ndarray, min_tape_frac: float = 0.03) -> "float | None":
    """Return yaw_deg (compatible with yaw_sign=-1 pipeline) from tape grid angle.

    Applies HoughLines to the Canny edges of the orange-tape binary mask.
    Using Canny first is critical: applying HoughLines directly to the filled
    tape mask scatters votes incoherently across rho values and gives no clean
    peaks.  A high vote threshold (100) rejects short intersection-corner
    fragments and keeps only long tape-strip edges.

    HoughLines theta convention: theta is the normal angle (0=vertical, pi/2=horiz).
    For a tape line at angle α (image coords, CW positive):
        theta = pi/2 − α_rad
        deviation = median(theta) − pi/2 = −α_rad → yaw_deg = deviation_deg

    Returns None when the tape is not detectable.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    tape = cv2.inRange(hsv, (8, 80, 60), (28, 255, 255))
    k3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    tape = cv2.morphologyEx(tape, cv2.MORPH_CLOSE, k3)

    if float(tape.mean()) < 255.0 * min_tape_frac:
        return None

    # Canny on the tape mask → only tape-edge pixels enter HoughLines.
    edges = cv2.Canny(tape, 30, 90)

    lines = cv2.HoughLines(edges, rho=1, theta=np.pi / 720, threshold=100)
    if lines is None or len(lines) < 3:
        return None

    thetas = lines[:, 0, 1]  # normal angles in [0, pi]

    # Horizontal tape family: normal angle near pi/2 (= [45°, 135°]).
    horiz = thetas[(thetas > np.pi / 4) & (thetas < 3 * np.pi / 4)]
    if len(horiz) < 3:
        return None

    # deviation = median(theta) − pi/2 = − α_img (CW tilt of tape in image coords).
    # With yaw_sign=-1: rotation = −deviation_deg (CCW) corrects the tilt.
    alpha_deg = float(np.degrees(float(np.median(horiz)) - np.pi / 2))
    while alpha_deg > 45.0:
        alpha_deg -= 90.0
    while alpha_deg < -45.0:
        alpha_deg += 90.0

    return float(alpha_deg)


def dump_masks(bgr: np.ndarray, out_path) -> None:
    """Side-by-side debug image of valid/grid/obstacle/wall + edges, for tuning."""
    vm = valid_mask(bgr)
    gm = grid_mask(bgr)
    om = obstacle_mask(bgr)
    wm = wall_mask(bgr)
    em = edge_image(bgr)
    h, w = bgr.shape[:2]
    rows = []
    for label, m in [("valid", vm), ("grid", gm), ("obst", om), ("wall", wm), ("edge", em)]:
        m3 = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
        cv2.putText(m3, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        rows.append(m3)
    tiled = np.hstack([bgr] + rows)
    cv2.imwrite(str(out_path), tiled)
