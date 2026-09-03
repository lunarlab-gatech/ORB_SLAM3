#!/usr/bin/env python3
import numpy as np
import cv2
from pathlib import Path
from tqdm import tqdm

npy_dir = Path('/home/sgarimella34/multi-robot-coordination/hercules_datasets/ausenv_roadseq/Drone1/depth')
output_dir = Path('/home/sgarimella34/multi-robot-coordination/hercules_datasets/ausenv_roadseq/Drone1/depth_png16')
output_dir.mkdir(exist_ok=True)

npy_files = sorted(npy_dir.glob('*.npy'), key=lambda x: float(x.stem))

print(f"Converting {len(npy_files)} depth files from .npy (meters) to 16-bit PNG (millimeters)...")
for npy_file in tqdm(npy_files):
    depth_m = np.load(npy_file)
    depth_mm = depth_m * 1000.0  # meters to mm
    depth_mm[depth_mm > 65000] = 0  # Clip far/invalid depths
    depth_uint16 = depth_mm.astype(np.uint16)

    output_file = output_dir / f"{npy_file.stem}.png"
    cv2.imwrite(str(output_file), depth_uint16)

print("Conversion complete!")
