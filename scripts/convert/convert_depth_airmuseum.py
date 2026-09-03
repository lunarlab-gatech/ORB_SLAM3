#!/usr/bin/env python3
"""
Convert precomputed FoundationStereo depth .npy (meters) for one AirMuseum robot
to 16-bit PNG (millimeters), nearest-timestamp-matched to and named after the
already-exported rectified left-camera image timestamps (rgbd_airsim.cc builds
both rgb and depth paths from the same timestamps.txt entries, so filenames must
match exactly).

Usage:
    python3 convert_depth_airmuseum.py --robot robotA
    python3 convert_depth_airmuseum.py --robot robotB
"""
import argparse
import numpy as np
import cv2
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--robot", required=True, choices=["drone", "robotA", "robotB", "robotC"])
args = parser.parse_args()
ROBOT = args.robot

npy_dir = Path(f"/media/sgarimella34/T74/AirMuseum_dataset/Scenario5/results/FoundationStereo/{ROBOT}/depth")
ts_path = Path(f"/home/sgarimella34/slam/orbslam3_ws/datasets/AirMuseum/{ROBOT}/timestamps.txt")
output_dir = Path(f"/home/sgarimella34/slam/orbslam3_ws/datasets/AirMuseum/{ROBOT}/depth_png16")
output_dir.mkdir(parents=True, exist_ok=True)

npy_files = sorted(npy_dir.glob("*.npy"), key=lambda p: float(p.stem))
npy_ts = np.array([float(p.stem) for p in npy_files])
print(f"{len(npy_files)} depth npy files, ts range [{npy_ts[0]:.3f}, {npy_ts[-1]:.3f}]")

image_ts = [line.strip() for line in ts_path.read_text().splitlines() if line.strip()]
image_ts_f = np.array([float(t) for t in image_ts])
print(f"{len(image_ts)} image timestamps, ts range [{image_ts_f[0]:.3f}, {image_ts_f[-1]:.3f}]")

diffs = []
for img_t_str, img_t in zip(image_ts, image_ts_f):
    idx = np.searchsorted(npy_ts, img_t)
    candidates = [i for i in (idx - 1, idx) if 0 <= i < len(npy_ts)]
    best = min(candidates, key=lambda i: abs(npy_ts[i] - img_t))
    diff = abs(npy_ts[best] - img_t)
    diffs.append(diff)

    depth_m = np.load(npy_files[best])
    depth_mm = depth_m * 1000.0
    depth_mm[~np.isfinite(depth_mm)] = 0
    depth_mm[depth_mm > 65000] = 0
    depth_mm[depth_mm < 0] = 0
    depth_uint16 = depth_mm.astype(np.uint16)

    out_file = output_dir / f"{img_t_str}.png"
    cv2.imwrite(str(out_file), depth_uint16)

diffs = np.array(diffs)
print(f"Wrote {len(image_ts)} depth PNGs to {output_dir}")
print(f"Timestamp match offsets (s): max={diffs.max():.6f} mean={diffs.mean():.6f} "
      f"median={np.median(diffs):.6f}, >0.05s count={np.sum(diffs > 0.05)}")
