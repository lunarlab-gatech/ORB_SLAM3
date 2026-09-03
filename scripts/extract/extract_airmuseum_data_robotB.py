#!/usr/bin/env python3
"""
Extract AirMuseum Scenario5 stereo+GT data for robotB into the flat-file
format this workspace's ORB-SLAM3 RGB-D loader expects (timestamp-named PNGs +
timestamps.txt + TUM-format ground truth), for Mono-Depth mode.

Mirrors extract_airmuseum_data.py (drone), minus IMU extraction (not needed
for Mono-Depth) and minus right-image export (not needed for RGB-D mode,
but right-camera data is still loaded/synced since stereo rectification
needs both cameras).
"""
import numpy as np
from decimal import Decimal
from pathlib import Path
import yaml

from robotdataprocess import (
    ImageData, ImageDataOnDisk, OdometryData, CoordinateFrame,
    TransformationData, CameraData,
)

ROBOT_NAME = "robotB"
DATASET_PATH = Path("/media/sgarimella34/T74/AirMuseum_dataset/Scenario5")
LOCAL_SENSORS_DIR = Path("/home/sgarimella34/slam/orbslam3_ws/datasets/AirMuseum/sensors")
OUT_DIR = Path("/home/sgarimella34/slam/orbslam3_ws/datasets/AirMuseum") / ROBOT_NAME

base_path = DATASET_PATH / "data"
input_path = base_path / ROBOT_NAME

robot_left_cam_map = {'drone': 'cam100', 'robotA': 'cam101', 'robotB': 'cam101', 'robotC': 'cam101'}
cam_id_to_calib_name = {'cam100': 'cam0', 'cam101': 'cam1'}
cam_id_to_bag_name_map = {'cam100': 'cam100_imu.bag', 'cam101': 'cam101.bag'}
left_cam_id = robot_left_cam_map[ROBOT_NAME]
right_cam_id = 'cam101' if left_cam_id == 'cam100' else 'cam100'
left_bag = input_path / cam_id_to_bag_name_map[left_cam_id]
right_bag = input_path / cam_id_to_bag_name_map[right_cam_id]
print(f"left_cam_id={left_cam_id} ({left_bag.name}), right_cam_id={right_cam_id} ({right_bag.name})")

calib_name = ROBOT_NAME + "_cameras_calib.yaml"
calib_yaml_path = LOCAL_SENSORS_DIR / calib_name

print(f"=== Loading stereo calibration from {calib_yaml_path} ===")
cam_data_left, cam_data_right = CameraData.from_kalibr_stereo(
    calib_yaml_path, cam_id_to_calib_name[left_cam_id], cam_id_to_calib_name[right_cam_id], alpha=0.0)

print("=== Loading images from bags ===")
left_image_data = ImageDataOnDisk.from_ros1_bag(left_bag, f'/{ROBOT_NAME}/{left_cam_id}/image_raw')
right_image_data = ImageDataOnDisk.from_ros1_bag(right_bag, f'/{ROBOT_NAME}/{right_cam_id}/image_raw')
assert left_image_data.encoding == right_image_data.encoding, "Left/Right image encodings must match!"
assert left_image_data.encoding == ImageData.ImageEncoding.Mono8, "Expected AirMuseum imagery to be Mono8"
print(f"Left images: {left_image_data.len()}, Right images: {right_image_data.len()}")

CameraData.align_ImageData_and_CameraData_to_imu_ts([left_image_data], cam_data_left)
CameraData.align_ImageData_and_CameraData_to_imu_ts([right_image_data], cam_data_right)

ImageDataOnDisk.crop_to_matched(left_image_data, right_image_data, Decimal('0.01'))
print(f"After stereo sync: {left_image_data.len()} matched pairs")

ImageDataOnDisk.undistort_imagery_stereo(left_image_data, right_image_data, cam_data_left, cam_data_right)
print(f"Post-rectification D_left={cam_data_left.D}, D_right={cam_data_right.D}")

start, end = Decimal('0.0'), None
left_image_data.crop_data(start, end)
right_image_data.crop_data(start, end)
print(f"After crop: {left_image_data.len()} images")

# ==================================== Ground truth ====================================
print("=== Loading ground truth ===")
ground_truth = OdometryData.from_txt(
    input_path / "body_stamped_groundtruth.txt", 'world', 'imu', CoordinateFrame.NONE,
    True, [0, 1, 2, 3, 7, 4, 5, 6])
name_to_frame_map = {"drone": CoordinateFrame.FLU, "robotA": CoordinateFrame.UFL,
                      "robotB": CoordinateFrame.UFL, "robotC": CoordinateFrame.FUR}
ground_truth.redefine_local_axes(name_to_frame_map[ROBOT_NAME], CoordinateFrame.FLU)
ground_truth.crop_data(start, end)
print(f"GT poses: {ground_truth.len()}")

# ==================================== Export ====================================
OUT_DIR.mkdir(parents=True, exist_ok=True)
left_out = OUT_DIR / "rgb_stereo_left"
print(f"=== Writing rectified left images to {left_out} (right images not needed for RGB-D mode) ===")
left_image_data.to_image_files(left_out)

ts_path = OUT_DIR / "timestamps.txt"
with open(ts_path, 'w') as f:
    for t in left_image_data.timestamps:
        f.write(f"{t:.9f}\n")
print(f"Wrote {ts_path}")

gt_path = OUT_DIR / "gt_tum.txt"
ground_truth.to_tum(gt_path)
print(f"Wrote {gt_path}")

# ==================================== Config constants ====================================
print("\n" + "=" * 70)
print("CONFIG CONSTANTS (for RobotB_RGBD.yaml)")
print("=" * 70)

fx1, fy1, cx1, cy1 = cam_data_left.P[0, 0], cam_data_left.P[1, 1], cam_data_left.P[0, 2], cam_data_left.P[1, 2]
print(f"Camera1 (left): fx={fx1} fy={fy1} cx={cx1} cy={cy1}")
print(f"D_left (should be ~0): {cam_data_left.D}")
print(f"width={cam_data_left.width} height={cam_data_left.height}")

baseline = -cam_data_right.P[0, 3] / cam_data_right.P[0, 0]
print(f"\nBaseline: {baseline} m")

print(f"\nFinal counts: images={left_image_data.len()}, gt_poses={ground_truth.len()}")
print("DONE")
