"""Phase A: pose-prior tile placement on a canvas (Guide §4.1)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np

from .io_utils import Tile, WorldExtent


@dataclass
class PlaceConfig:
    ppm: float = 500.0
    margin_px: int = 250
    # fov_v_deg=43 matches the real Tello (horizontal_fov=0.96 rad in the SDF
    # -> ~55° horizontal at 960x720 -> ~43° vertical). Override via CLI if the
    # capture rig changes.
    fov_v_deg: float = 43.0
    # Yaw application convention, validated by debug_yaw.py on the 16fotos dataset:
    # rotating each tile by +yaw_deg (yaw_sign=+1, no axis offset) aligned grid
    # lines across seams. Re-run debug_yaw if the world/image axis convention changes.
    yaw_sign: float = 1.0
    yaw_axis_offset_deg: float = 0.0


@dataclass
class CanvasSpec:
    width: int
    height: int
    extent: WorldExtent
    cfg: PlaceConfig

    def world_to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        # Flip X: world X+ -> canvas left (x_min -> right).
        # Keep Y: world Y+ -> canvas down (y_min -> top).
        # Photo 0 (x=x_min, y=y_max) -> bottom-right. Photo 15 (x=x_min, y=y_min) -> top-right.
        cx = (self.extent.x_max - x) * self.cfg.ppm + self.cfg.margin_px
        cy = (y - self.extent.y_min) * self.cfg.ppm + self.cfg.margin_px
        return cx, cy


@dataclass
class PlacedTile:
    idx: int
    warped_bgr: np.ndarray            # warped tile centered on its rotated bbox
    warped_valid: np.ndarray          # uint8 mask, same shape
    warped_feather: np.ndarray        # float32 feather weights, same shape
    cx: float                         # canvas center x (pixel)
    cy: float                         # canvas center y (pixel)
    footprint_px: int                 # square pixel footprint before rotation
    rotation_deg: float = 0.0         # rotation applied during warp (used by tape_snap)


def make_canvas_spec(extent: WorldExtent, cfg: PlaceConfig, footprint_px_max: int) -> CanvasSpec:
    # Add room for the tile footprint at corner waypoints.
    # Canvas X maps to world X (dx), canvas Y maps to world Y (dy).
    half_fp = footprint_px_max // 2 + 32
    width = int(round(extent.dx * cfg.ppm + 2 * cfg.margin_px + 2 * half_fp))
    height = int(round(extent.dy * cfg.ppm + 2 * cfg.margin_px + 2 * half_fp))
    return CanvasSpec(width=width, height=height, extent=extent, cfg=cfg)


def _radial_feather(h: int, w: int) -> np.ndarray:
    yy, xx = np.mgrid[:h, :w].astype(np.float32)
    cx, cy = w / 2.0, h / 2.0
    r2 = (xx - cx) ** 2 + (yy - cy) ** 2
    sigma2 = 0.4 * (w * w + h * h) / 4.0
    return np.exp(-r2 / sigma2).astype(np.float32)


def footprint_pixels(z: float, img_h: int, fov_v_deg: float, ppm: float) -> int:
    fov_v = math.radians(fov_v_deg)
    footprint_m = 2.0 * z * math.tan(fov_v / 2.0)
    return max(8, int(round(footprint_m * ppm)))


def _rotate_full(img: np.ndarray, angle_deg: float, flags: int, border_value=0) -> np.ndarray:
    h, w = img.shape[:2]
    cx, cy = w / 2.0, h / 2.0
    M = cv2.getRotationMatrix2D((cx, cy), angle_deg, 1.0)
    cos = abs(M[0, 0]); sin = abs(M[0, 1])
    new_w = int(h * sin + w * cos)
    new_h = int(h * cos + w * sin)
    M[0, 2] += new_w / 2.0 - cx
    M[1, 2] += new_h / 2.0 - cy
    return cv2.warpAffine(
        img, M, (new_w, new_h),
        flags=flags, borderMode=cv2.BORDER_CONSTANT, borderValue=border_value,
    )


def prepare_tile(tile: Tile, cfg: PlaceConfig, preproc_bgr: np.ndarray = None) -> PlacedTile:
    """Resize for z, rotate for yaw, build feather. Caller decides where on the canvas."""
    src = preproc_bgr if preproc_bgr is not None else tile.img
    fp = footprint_pixels(tile.z, tile.img_h, cfg.fov_v_deg, cfg.ppm)

    # Resize so that the *vertical* image extent maps to footprint_px on canvas.
    scale = fp / float(tile.img_h)
    new_w = max(8, int(round(tile.img_w * scale)))
    new_h = fp
    resized = cv2.resize(src, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # Valid mask before rotation (computed on the resized BGR).
    from .preprocess import valid_mask as _vm
    vm = _vm(resized)
    feather = _radial_feather(new_h, new_w) * (vm.astype(np.float32) / 255.0)

    angle = cfg.yaw_sign * tile.yaw_deg + cfg.yaw_axis_offset_deg
    warped_bgr = _rotate_full(resized, angle, cv2.INTER_LINEAR, border_value=(0, 0, 0))
    warped_valid = _rotate_full(vm, angle, cv2.INTER_NEAREST, border_value=0)
    warped_feather = _rotate_full(feather, angle, cv2.INTER_LINEAR, border_value=0.0)

    return PlacedTile(
        idx=tile.idx,
        warped_bgr=warped_bgr,
        warped_valid=warped_valid,
        warped_feather=warped_feather,
        cx=0.0, cy=0.0,
        footprint_px=fp,
        rotation_deg=angle,
    )


def assign_canvas_centers(tiles: List[Tile], spec: CanvasSpec, placed: List[PlacedTile]) -> None:
    """Fill in placed[i].cx/cy from the world poses."""
    for t, p in zip(tiles, placed):
        cx, cy = spec.world_to_canvas(t.x, t.y)
        p.cx, p.cy = cx, cy


def paste_phase_a(spec: CanvasSpec, placed: List[PlacedTile]) -> Tuple[np.ndarray, np.ndarray]:
    """Feather-blend the placed tiles into an RGB canvas. Returns (canvas_bgr_uint8, weight)."""
    H, W = spec.height, spec.width
    accum = np.zeros((H, W, 3), dtype=np.float32)
    weight = np.zeros((H, W), dtype=np.float32)
    for p in placed:
        h, w = p.warped_bgr.shape[:2]
        x0 = int(round(p.cx - w / 2.0))
        y0 = int(round(p.cy - h / 2.0))
        x1, y1 = x0 + w, y0 + h
        # Clip to canvas.
        sx0 = max(0, -x0); sy0 = max(0, -y0)
        dx0 = max(0, x0);  dy0 = max(0, y0)
        dx1 = min(W, x1);  dy1 = min(H, y1)
        if dx1 <= dx0 or dy1 <= dy0:
            continue
        sw = dx1 - dx0; sh = dy1 - dy0
        fw = p.warped_feather[sy0:sy0 + sh, sx0:sx0 + sw]
        accum[dy0:dy1, dx0:dx1] += p.warped_bgr[sy0:sy0 + sh, sx0:sx0 + sw].astype(np.float32) * fw[..., None]
        weight[dy0:dy1, dx0:dx1] += fw
    canvas = accum / np.maximum(weight[..., None], 1e-6)
    return canvas.clip(0, 255).astype(np.uint8), weight


def annotate_indices(canvas: np.ndarray, placed: List[PlacedTile]) -> np.ndarray:
    out = canvas.copy()
    for p in placed:
        txt = f"wp_{p.idx:02d}"
        org = (int(p.cx) - 30, int(p.cy))
        cv2.putText(out, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.circle(out, (int(p.cx), int(p.cy)), 4, (0, 0, 255), -1)
    return out
