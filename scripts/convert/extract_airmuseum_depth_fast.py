#!/usr/bin/env python3
"""
Convert Fast-FoundationStereo depth .npy (meters, native 480x320) to 16-bit
PNG (millimeters) resized to match the rectified RGB resolution (640x512),
using robotdataprocess's new ImageDataOnDisk.resize(), then nearest-
timestamp-matched to and named after the already-exported rectified
left-camera image timestamps.

Usage:
    python3 extract_airmuseum_depth_fast.py --robot drone
    python3 extract_airmuseum_depth_fast.py --robot robotA
"""
import argparse
import numpy as np
import cv2
from pathlib import Path
from robotdataprocess.data_types.ImageData.ImageDataOnDisk import ImageDataOnDisk

# cam100 for drone, cam101 for the ground robots
CAM_ID_BY_ROBOT = {
    "drone": "cam100_depth",
    "robotA": "cam101_depth",
    "robotB": "cam101_depth",
    "robotC": "cam101_depth",
}

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--robot", default="drone", choices=list(CAM_ID_BY_ROBOT))
args = parser.parse_args()
ROBOT = args.robot

NPY_DIR = Path(f"/media/sgarimella34/T74/AirMuseum_dataset/Scenario5/results/Fast-FoundationStereo/{ROBOT}/depth")
TS_PATH = Path(f"/home/sgarimella34/slam/orbslam3_ws/datasets/AirMuseum/{ROBOT}/timestamps.txt")
OUTPUT_DIR = Path(f"/home/sgarimella34/slam/orbslam3_ws/datasets/AirMuseum/{ROBOT}/depth_png16_fast")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

RGB_HEIGHT, RGB_WIDTH = 512, 640

depth_data = ImageDataOnDisk.from_npy_files(NPY_DIR, f"{ROBOT}/{CAM_ID_BY_ROBOT[ROBOT]}")
print(f"Loaded {len(depth_data.images)} depth frames, native {depth_data.width}x{depth_data.height}")
depth_data.resize(RGB_HEIGHT, RGB_WIDTH)
print(f"After resize(): {depth_data.width}x{depth_data.height}")

npy_ts = np.array([float(t) for t in depth_data.timestamps])

image_ts = [line.strip() for line in TS_PATH.read_text().splitlines() if line.strip()]
image_ts_f = np.array([float(t) for t in image_ts])
print(f"{len(image_ts)} image timestamps, ts range [{image_ts_f[0]:.3f}, {image_ts_f[-1]:.3f}]")

diffs = []
for img_t_str, img_t in zip(image_ts, image_ts_f):
    idx = np.searchsorted(npy_ts, img_t)
    candidates = [i for i in (idx - 1, idx) if 0 <= i < len(npy_ts)]
    best = min(candidates, key=lambda i: abs(npy_ts[i] - img_t))
    diff = abs(npy_ts[best] - img_t)
    diffs.append(diff)

    depth_m = depth_data.images[best]
    depth_mm = depth_m.astype(np.float64) * 1000.0
    depth_mm[~np.isfinite(depth_mm)] = 0
    depth_mm[depth_mm > 65000] = 0
    depth_mm[depth_mm < 0] = 0
    depth_uint16 = depth_mm.astype(np.uint16)

    assert depth_uint16.shape == (RGB_HEIGHT, RGB_WIDTH), depth_uint16.shape

    out_file = OUTPUT_DIR / f"{img_t_str}.png"
    cv2.imwrite(str(out_file), depth_uint16)

diffs = np.array(diffs)
print(f"Wrote {len(image_ts)} depth PNGs to {OUTPUT_DIR}")
print(f"Timestamp match offsets: max={diffs.max():.6f}s, mean={diffs.mean():.6f}s, >0.05s count={np.sum(diffs > 0.05)}")
