"""Tile + pose loading for the 16-waypoint mission dataset."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import List

import cv2
import numpy as np


@dataclass
class Tile:
    idx: int
    img: np.ndarray
    x: float
    y: float
    z: float
    yaw_deg: float
    pose_src: str
    img_w: int
    img_h: int
    path: Path


@dataclass
class WorldExtent:
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def dx(self) -> float:
        return self.x_max - self.x_min

    @property
    def dy(self) -> float:
        return self.y_max - self.y_min


def load_tiles(folder: Path | str) -> List[Tile]:
    folder = Path(folder)
    json_files = sorted(folder.glob("wp_*.json"))
    if not json_files:
        raise FileNotFoundError(f"No wp_*.json files found in {folder}")

    tiles: List[Tile] = []
    for jf in json_files:
        with open(jf) as f:
            d = json.load(f)
        img_path = jf.with_suffix(".png")
        if not img_path.exists():
            raise FileNotFoundError(f"Missing image for {jf.name}: {img_path}")
        img = cv2.imread(str(img_path))
        if img is None:
            raise RuntimeError(f"Failed to read {img_path}")
        img = cv2.flip(img, 1)  # Tello camera produces horizontally-mirrored images

        pose_src = d.get("pose_src", "")
        if "optitrack_yaw" not in pose_src:
            print(
                f"[io_utils] WARN {jf.name}: pose_src='{pose_src}' "
                f"does not include 'optitrack_yaw' — yaw may be stale."
            )

        tiles.append(
            Tile(
                idx=int(d["wp_index"]),
                img=img,
                x=float(d["x"]),
                y=float(d["y"]),
                z=float(d["z"]),
                yaw_deg=float(d.get("yaw_deg", 0.0)),
                pose_src=pose_src,
                img_w=int(d.get("img_w", img.shape[1])),
                img_h=int(d.get("img_h", img.shape[0])),
                path=img_path,
            )
        )

    tiles.sort(key=lambda t: t.idx)
    return tiles


def world_extent(tiles: List[Tile]) -> WorldExtent:
    xs = [t.x for t in tiles]
    ys = [t.y for t in tiles]
    return WorldExtent(min(xs), max(xs), min(ys), max(ys))
