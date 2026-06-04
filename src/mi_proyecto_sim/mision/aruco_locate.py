"""ArUco marker localization: detect in tiles, project to world coordinates."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import cv2
import numpy as np

from .io_utils import Tile
from .place_pose import CanvasSpec, PlacedTile, footprint_pixels


_DICTS_TO_TRY = [
    ("4x4_50", cv2.aruco.DICT_4X4_50),
]

# Minimum apparent marker side length in image pixels.  A real floor marker of
# ~10cm physical size at 2m altitude with 43° vertical FOV appears as ~46px.
# Anything below this is almost certainly noise from the decoder.
_MIN_MARKER_PX = 20


def _strict_detector() -> cv2.aruco.ArucoDetector:
    """Return a detector with OpenCV default (strict) parameters.

    The relaxed params used in scan_aruco.py (minPerimRate=0.01,
    polygonalApprox=0.1) are good for discovery but generate false positives
    from noise.  OpenCV defaults (minPerimRate=0.03, polygonalApprox=0.03)
    require well-formed quadrilateral corners, which random texture patches
    rarely satisfy.
    """
    try:
        p = cv2.aruco.DetectorParameters()  # OpenCV 4.7+
    except AttributeError:
        p = cv2.aruco.DetectorParameters_create()  # OpenCV 4.6 and older
    return p


@dataclass
class ArUcoSighting:
    marker_id: int
    tile_idx: int
    canvas_x: float
    canvas_y: float
    world_x: float
    world_y: float
    pixel_size: float
    dict_name: str


@dataclass
class ArUcoMarker:
    id: int
    world_x: float
    world_y: float
    canvas_x: float
    canvas_y: float
    n_detections: int
    tiles_seen: List[int] = field(default_factory=list)


def detect_in_tile(
    tile: Tile,
    placed: PlacedTile,
    spec: CanvasSpec,
) -> List[ArUcoSighting]:
    """Detect ArUco markers in one tile and return world + canvas positions.

    Uses strict (default) OpenCV detector parameters to avoid false positives,
    then rejects any detection whose apparent marker size is below _MIN_MARKER_PX
    — a physically-grounded threshold based on expected drone altitude.
    """
    img = tile.img  # already horizontally flipped by io_utils
    img_h, img_w = img.shape[:2]

    fp = footprint_pixels(tile.z, img_h, spec.cfg.fov_v_deg, spec.cfg.ppm)
    alpha_rad = math.radians(
        spec.cfg.yaw_sign * placed.rotation_deg  # rotation_deg = yaw_sign*yaw_deg+offset
    )
    cos_a, sin_a = math.cos(alpha_rad), math.sin(alpha_rad)

    strict_params = _strict_detector()
    sightings: List[ArUcoSighting] = []
    seen_ids: set = set()

    for dict_name, dict_id in _DICTS_TO_TRY:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        detector = cv2.aruco.ArucoDetector(aruco_dict, strict_params)
        corners_list, ids, _ = detector.detectMarkers(img)

        if ids is None:
            continue

        for marker_id, corners in zip(ids.flatten(), corners_list):
            if int(marker_id) in seen_ids:
                continue

            center = corners[0].mean(axis=0)
            px, py = float(center[0]), float(center[1])

            sides = [
                np.linalg.norm(corners[0][i] - corners[0][(i + 1) % 4])
                for i in range(4)
            ]
            pixel_size = float(np.mean(sides))

            # Reject detections that are too small to be real floor markers.
            if pixel_size < _MIN_MARKER_PX:
                continue

            seen_ids.add(int(marker_id))

            # Offset from image center → resized image pixels
            dx_res = (px - img_w / 2.0) * fp / img_h
            dy_res = (py - img_h / 2.0) * fp / img_h

            # Rotate by yaw (cv2.getRotationMatrix2D convention used in _rotate_full)
            dx_rot = dx_res * cos_a + dy_res * sin_a
            dy_rot = -dx_res * sin_a + dy_res * cos_a

            canvas_x = placed.cx + dx_rot
            canvas_y = placed.cy + dy_rot

            world_x = spec.extent.x_max - (canvas_x - spec.cfg.margin_px) / spec.cfg.ppm
            world_y = spec.extent.y_min + (canvas_y - spec.cfg.margin_px) / spec.cfg.ppm

            sightings.append(ArUcoSighting(
                marker_id=int(marker_id),
                tile_idx=tile.idx,
                canvas_x=canvas_x,
                canvas_y=canvas_y,
                world_x=world_x,
                world_y=world_y,
                pixel_size=pixel_size,
                dict_name=dict_name,
            ))

    return sightings


def locate_aruco_markers(
    tiles: List[Tile],
    placed: List[PlacedTile],
    spec: CanvasSpec,
    debug: bool = False,
) -> List[ArUcoMarker]:
    """Detect ArUco in all tiles and return median world position per marker ID."""
    placed_by_idx: Dict[int, PlacedTile] = {p.idx: p for p in placed}
    all_sightings: Dict[int, List[ArUcoSighting]] = {}

    for tile in tiles:
        p = placed_by_idx[tile.idx]
        for s in detect_in_tile(tile, p, spec):
            all_sightings.setdefault(s.marker_id, []).append(s)
            if debug:
                print(
                    f"  [aruco] wp_{tile.idx:02d}: id={s.marker_id} dict={s.dict_name} "
                    f"world=({s.world_x:.3f},{s.world_y:.3f}) "
                    f"canvas=({s.canvas_x:.0f},{s.canvas_y:.0f}) "
                    f"size={s.pixel_size:.1f}px"
                )

    markers: List[ArUcoMarker] = []
    for marker_id, sightings in sorted(all_sightings.items()):
        world_x = float(np.median([s.world_x for s in sightings]))
        world_y = float(np.median([s.world_y for s in sightings]))
        canvas_x = float(np.median([s.canvas_x for s in sightings]))
        canvas_y = float(np.median([s.canvas_y for s in sightings]))
        markers.append(ArUcoMarker(
            id=marker_id,
            world_x=world_x,
            world_y=world_y,
            canvas_x=canvas_x,
            canvas_y=canvas_y,
            n_detections=len(sightings),
            tiles_seen=sorted({s.tile_idx for s in sightings}),
        ))

    return markers


def draw_aruco_overlay(
    base_img: np.ndarray,
    markers: List[ArUcoMarker],
    crop_offset: Tuple[int, int] = (0, 0),
) -> np.ndarray:
    """Overlay ArUco marker positions on an occupancy map or mosaic image."""
    out = (
        cv2.cvtColor(base_img, cv2.COLOR_GRAY2BGR)
        if base_img.ndim == 2
        else base_img.copy()
    )
    cx_off, cy_off = crop_offset

    for m in markers:
        cx = int(round(m.canvas_x - cx_off))
        cy = int(round(m.canvas_y - cy_off))

        cv2.drawMarker(out, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 30, 3, cv2.LINE_AA)
        cv2.circle(out, (cx, cy), 15, (0, 0, 255), 2, cv2.LINE_AA)

        id_txt = f"id{m.id}"
        cv2.putText(out, id_txt, (cx + 18, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, id_txt, (cx + 18, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2, cv2.LINE_AA)

        coord_txt = f"({m.world_x:.2f},{m.world_y:.2f})"
        cv2.putText(out, coord_txt, (cx + 18, cy + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, coord_txt, (cx + 18, cy + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1, cv2.LINE_AA)

    return out
