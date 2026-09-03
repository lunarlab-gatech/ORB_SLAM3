#!/usr/bin/env python3
"""
Extract AirMuseum Scenario5 stereo+IMU+GT data for one robot into the flat-file
format this workspace's ORB-SLAM3 loaders expect (timestamp-named PNGs +
timestamps.txt + space-separated IMU txt + TUM-format ground truth).

Ported from `load_AirMuseum_data` (lunarlab-gatech/MeronomyGraph,
branch feature/ablation-hash-cache, research/run_multi_robot_slam.py:266-401)
through the ground-truth-loading line; the two ROMAN-glue sections after that
("Convert to robotdatapy format" / "Package into DataParams for ROMAN") are
dropped since we feed ORB-SLAM3 directly instead of ROMAN.

Depth loading is skipped entirely -- not needed for Stereo-Inertial mode.
IMU extraction is custom, since robotdataprocess's ImuData has no ROS1-bag
reader (only ROS2-bag and txt-file).
"""
import numpy as np
from decimal import Decimal
from pathlib import Path
import yaml

from robotdataprocess import (
    ImageData, ImageDataOnDisk, OdometryData, CoordinateFrame,
    TransformationData, CameraData, ImuData,
)
from rosbags.rosbag1 import Reader as Reader1
from rosbags.typesys import Stores, get_typestore

ROBOT_NAME = "drone"
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

calib_name = ROBOT_NAME + "_cameras_calib.yaml"
calib_yaml_path = LOCAL_SENSORS_DIR / calib_name

print(f"=== Loading stereo calibration from {calib_yaml_path} ===")
cam_data_left, cam_data_right = CameraData.from_kalibr_stereo(
    calib_yaml_path, cam_id_to_calib_name[left_cam_id], cam_id_to_calib_name[right_cam_id], alpha=0.0)

# Snapshot the rectifying rotation for cam0 BEFORE undistort_imagery_stereo resets camera_data.R to identity.
R1_rectify = cam_data_left.R.copy()

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

# ==================================== Custom IMU extraction ====================================
# ImuData has no from_ros1_bag; pattern-matched off CameraData.from_ros1_bag /
# ImageDataOnDisk.from_ros1_bag's ROS1_NOETIC typestore + header-stamp usage.
def load_imu_ros1(bag_path, topic):
    typestore = get_typestore(Stores.ROS1_NOETIC)
    timestamps, lin_acc, ang_vel = [], [], []
    with Reader1(Path(bag_path)) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise ValueError(f"Topic {topic!r} not found in {bag_path}")
        msgtype = conns[0].msgtype
        for _, _, rawdata in reader.messages(connections=conns):
            msg = typestore.deserialize_ros1(rawdata, msgtype)
            stamp = msg.header.stamp
            t = Decimal(stamp.sec) + Decimal(stamp.nanosec) * Decimal('1e-9')
            timestamps.append(t)
            lin_acc.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            ang_vel.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    order = np.argsort(timestamps)
    timestamps = [timestamps[i] for i in order]
    lin_acc = np.array(lin_acc)[order]
    ang_vel = np.array(ang_vel)[order]
    return timestamps, lin_acc, ang_vel

print("=== Loading IMU from bag ===")
imu_ts, imu_acc, imu_gyro = load_imu_ros1(left_bag, f'/{ROBOT_NAME}/imu')
print(f"IMU samples (raw): {len(imu_ts)}")

imu_data = ImuData(f'{ROBOT_NAME}_imu', CoordinateFrame.FLU, imu_ts, imu_acc, imu_gyro, None)
imu_data.crop_data(start, end)
print(f"IMU samples after crop: {imu_data.len()}")

# ==================================== Export ====================================
OUT_DIR.mkdir(parents=True, exist_ok=True)
left_out = OUT_DIR / "rgb_stereo_left"
right_out = OUT_DIR / "rgb_stereo_right"
print(f"=== Writing images to {left_out}, {right_out} ===")
left_image_data.to_image_files(left_out)
right_image_data.to_image_files(right_out)

ts_path = OUT_DIR / "timestamps.txt"
with open(ts_path, 'w') as f:
    for t in left_image_data.timestamps:
        f.write(f"{t:.9f}\n")
print(f"Wrote {ts_path}")

imu_path = OUT_DIR / "imu.txt"
with open(imu_path, 'w') as f:
    for i in range(imu_data.len()):
        t = imu_data.timestamps[i]
        a = imu_data.lin_acc[i]
        g = imu_data.ang_vel[i]
        f.write(f"{t} {a[0]} {a[1]} {a[2]} {g[0]} {g[1]} {g[2]}\n")
print(f"Wrote {imu_path}")

gt_path = OUT_DIR / "gt_tum.txt"
ground_truth.to_tum(gt_path)
print(f"Wrote {gt_path}")

# ==================================== Config constants ====================================
print("\n" + "=" * 70)
print("CONFIG CONSTANTS (for Phase-2 ORB-SLAM3 yaml)")
print("=" * 70)

fx1, fy1, cx1, cy1 = cam_data_left.P[0, 0], cam_data_left.P[1, 1], cam_data_left.P[0, 2], cam_data_left.P[1, 2]
fx2, fy2, cx2, cy2 = cam_data_right.P[0, 0], cam_data_right.P[1, 1], cam_data_right.P[0, 2], cam_data_right.P[1, 2]
print(f"Camera1 (left):  fx={fx1} fy={fy1} cx={cx1} cy={cy1}")
print(f"Camera2 (right): fx={fx2} fy={fy2} cx={cx2} cy={cy2}")
print(f"D_left (should be ~0): {cam_data_left.D}")
print(f"D_right (should be ~0): {cam_data_right.D}")
print(f"width={cam_data_left.width} height={cam_data_left.height}")

baseline = -cam_data_right.P[0, 3] / cam_data_right.P[0, 0]
print(f"\nBaseline: {baseline} m")
T_c1_c2 = np.eye(4)
T_c1_c2[0, 3] = baseline
print(f"Stereo.T_c1_c2:\n{T_c1_c2}")

# IMU.T_b_c1 for the RECTIFIED cam0 frame.
# Kalibr's T_cam_imu (M) maps IMU-frame points into the ORIGINAL cam0 frame: p_origcam0 = M @ p_imu.
# OpenCV's stereoRectify R1 maps original-cam0 points into the rectified-cam0 frame: p_rectcam0 = R1 @ p_origcam0.
# So T_rectcam0_imu = R1_4x4 @ M, and IMU.T_b_c1 (rectified cam0 -> imu) = inverse(T_rectcam0_imu).
M = TransformationData.from_kalibr(calib_yaml_path, "cam0", "T_cam_imu", CoordinateFrame.FLU).as_matrix()
print(f"\nRaw T_cam0_imu (kalibr, unrectified):\n{M}")
R1_4x4 = np.eye(4)
R1_4x4[:3, :3] = R1_rectify
T_rectcam0_imu = R1_4x4 @ M
T_b_c1 = np.linalg.inv(T_rectcam0_imu)
print(f"\nIMU.T_b_c1 (rectified cam0 -> imu):\n{T_b_c1}")
residual = T_b_c1 @ T_rectcam0_imu
print(f"\nResidual check (T_b_c1 @ T_rectcam0_imu, should be ~I):\n{residual}")
print(f"Max abs deviation from identity: {np.max(np.abs(residual - np.eye(4)))}")

with open(LOCAL_SENSORS_DIR / "imu.yaml") as f:
    imu_yaml = yaml.safe_load(f)
print(f"\nIMU noise params from sensors/imu.yaml: {imu_yaml}")

print(f"\nFinal counts: images={left_image_data.len()}, imu={imu_data.len()}, gt_poses={ground_truth.len()}")
print("DONE")
