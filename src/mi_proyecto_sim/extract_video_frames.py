#!/usr/bin/env python3
"""
Extract frames from a video and pair them with telemetry pose data.
Produces wp_NN.png + wp_NN.json compatible with mision/stitch_pose.py.

Usage:
    python3 extract_video_frames.py \
        --video ~/Downloads/video-no20/scan.mp4 \
        --telemetry ~/Downloads/video-no20/telemetry.csv \
        --output mision_output/video_no20 \
        --min-dist 0.05

NOTE on flipping: load_tiles() in io_utils.py flips every image horizontally
(designed for Tello's mirrored lens). If this video is NOT from a mirrored
camera, pass --pre-flip so the double-flip yields correct orientation.
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import cv2


def load_telemetry(path: str) -> dict:
    rows = {}
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            fid = int(row["frame_id"])
            rows[fid] = {
                "timestamp_sec": float(row["timestamp_sec"]),
                "x": float(row["pos_x"]),
                "y": float(row["pos_y"]),
                "z": float(row["pos_z"]),
                "yaw_deg": math.degrees(float(row["yaw"])),
            }
    return rows


def main():
    parser = argparse.ArgumentParser(description="Extract video frames linked to telemetry pose")
    parser.add_argument("--video", required=True, help="Path to input MP4 video")
    parser.add_argument("--telemetry", required=True, help="Path to telemetry CSV")
    parser.add_argument("--output", required=True, help="Output directory for wp_NN.{png,json}")
    parser.add_argument(
        "--min-dist", type=float, default=0.05,
        help="Min XY distance (m) between saved frames for spatial subsampling (default: 0.05)",
    )
    parser.add_argument(
        "--every-n", type=int, default=None,
        help="Override spatial filter: save every Nth video frame instead",
    )
    parser.add_argument(
        "--pre-flip", action="store_true",
        help="Flip frames horizontally before saving (compensates for load_tiles() auto-flip)",
    )
    parser.add_argument(
        "--max-frames", type=int, default=None,
        help="Stop after saving this many frames",
    )
    parser.add_argument(
        "--end-time", type=float, default=None,
        help="Stop at this timestamp in seconds (e.g. 92 for 1:32)",
    )
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading telemetry from {args.telemetry} ...", flush=True)
    telemetry = load_telemetry(args.telemetry)
    print(f"  {len(telemetry)} telemetry rows loaded", flush=True)

    cap = cv2.VideoCapture(str(Path(args.video).expanduser()))
    if not cap.isOpened():
        sys.exit(f"ERROR: cannot open video {args.video}")

    total_video_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Video: {total_video_frames} frames @ {fps:.1f} fps", flush=True)
    print(f"Subsampling: {'every-n=' + str(args.every_n) if args.every_n else 'min-dist=' + str(args.min_dist) + 'm'}", flush=True)

    saved = 0
    last_xy = None
    frame_num = 1  # 1-indexed to match telemetry frame_id

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if args.end_time is not None and (frame_num - 1) / fps > args.end_time:
            break

        pose = telemetry.get(frame_num)
        if pose is not None:
            if args.every_n is not None:
                should_save = (frame_num % args.every_n == 0)
            else:
                if last_xy is None:
                    should_save = True
                else:
                    dx = pose["x"] - last_xy[0]
                    dy = pose["y"] - last_xy[1]
                    should_save = math.hypot(dx, dy) >= args.min_dist

            if should_save:
                if args.pre_flip:
                    frame = cv2.flip(frame, 1)

                idx = saved
                img_path = output_dir / f"wp_{idx:02d}.png"
                cv2.imwrite(str(img_path), cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

                meta = {
                    "wp_index": idx,
                    "x": pose["x"],
                    "y": pose["y"],
                    "z": pose["z"],
                    "yaw_deg": pose["yaw_deg"],
                    "pose_src": "video_telemetry",
                    "stamp": f"{pose['timestamp_sec']:.3f}",
                    "img_w": frame.shape[1],
                    "img_h": frame.shape[0],
                }
                with open(output_dir / f"wp_{idx:02d}.json", "w") as jf:
                    json.dump(meta, jf, indent=2)

                last_xy = (pose["x"], pose["y"])
                saved += 1
                print(f"\r  Saved {saved} frames (video frame {frame_num}/{total_video_frames})", end="", flush=True)

                if args.max_frames and saved >= args.max_frames:
                    break

        frame_num += 1

    cap.release()
    print(f"\nDone — {saved} frames saved to {output_dir}")


if __name__ == "__main__":
    main()
