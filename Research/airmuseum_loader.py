#!/usr/bin/env python3
"""
Shared AirMuseum Scenario5 stereo+IMU+GT loading logic for one robot (all four
robots -- drone, robotA, robotB, robotC -- run Stereo-Inertial).

Used by `run_airmuseum.py`, which feeds the loaded data straight into
ORB-SLAM3 in memory via the `orbslam3_python` pybind11 module -- no flat-file
round trip.

Ported from `load_AirMuseum_data` (lunarlab-gatech/MeronomyGraph,
branch feature/ablation-hash-cache, research/run_multi_robot_slam.py:266-401)
through the ground-truth-loading line; the two ROMAN-glue sections after that
("Convert to robotdatapy format" / "Package into DataParams for ROMAN") are
dropped since we feed ORB-SLAM3 directly instead of ROMAN.
"""
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path

import numpy as np

from robotdataprocess import (
    ImageData, ImageDataOnDisk, OdometryData, CoordinateFrame, CameraData, ImuData,
)

ROBOTS = ["drone", "robotA", "robotB", "robotC"]

REPO_ROOT = Path(__file__).resolve().parents[1]
DATASET_PATH = Path.home() / "data" / "AirMuseum_dataset" / "Scenario5"

ROBOT_LEFT_CAM = {'drone': 'cam100', 'robotA': 'cam101', 'robotB': 'cam101', 'robotC': 'cam101'}
CAM_ID_TO_CALIB_NAME = {'cam100': 'cam0', 'cam101': 'cam1'}
CAM_ID_TO_BAG_NAME = {'cam100': 'cam100_imu.bag', 'cam101': 'cam101.bag'}
ROBOT_TO_GT_FRAME = {"drone": CoordinateFrame.FLU, "robotA": CoordinateFrame.UFL,
                      "robotB": CoordinateFrame.UFL, "robotC": CoordinateFrame.FUR}


@dataclass
class AirMuseumRobotData:
    robot: str
    left_cam_id: str
    right_cam_id: str
    calib_yaml_path: Path
    cam_data_left: CameraData
    cam_data_right: CameraData
    left_rectify_R: np.ndarray  # cam_data_left.R, snapshotted before undistort_imagery_stereo resets it
    left_image_data: ImageDataOnDisk
    right_image_data: ImageDataOnDisk
    imu_data: ImuData
    ground_truth: OdometryData


def load_robot_data(robot: str, sensors_dir: Path, start: Decimal = Decimal('0.0'), end=None) -> AirMuseumRobotData:
    """
    Loads, syncs, rectifies, and crops one AirMuseum robot's stereo+IMU+GT data.

    Images are returned as `ImageDataOnDisk`, so nothing is decoded/rectified
    until a caller actually indexes `.images[i]` (or calls `.to_image_files()`).

    `sensors_dir` must contain each robot's Kalibr stereo+IMU calibration,
    named `<robot>_cameras_calib.yaml`.
    """
    if robot not in ROBOTS:
        raise ValueError(f"Unknown robot {robot!r}, expected one of {ROBOTS}")

    input_path = DATASET_PATH / "data" / robot

    left_cam_id = ROBOT_LEFT_CAM[robot]
    right_cam_id = 'cam101' if left_cam_id == 'cam100' else 'cam100'
    left_bag = input_path / CAM_ID_TO_BAG_NAME[left_cam_id]
    right_bag = input_path / CAM_ID_TO_BAG_NAME[right_cam_id]
    # The IMU topic only lives in the cam100_imu.bag recording, regardless of
    # which side (left/right) cam100 is assigned to for this robot (it's left
    # for the drone, right for the ground robots).
    imu_bag = input_path / CAM_ID_TO_BAG_NAME['cam100']

    calib_yaml_path = Path(sensors_dir) / f"{robot}_cameras_calib.yaml"

    print(f"=== Loading stereo calibration from {calib_yaml_path} (left={left_cam_id}, right={right_cam_id}) ===")
    cam_data_left, cam_data_right = CameraData.from_kalibr_stereo(
        calib_yaml_path, CAM_ID_TO_CALIB_NAME[left_cam_id], CAM_ID_TO_CALIB_NAME[right_cam_id], alpha=0.0)

    # Snapshot the left camera's rectifying rotation BEFORE undistort_imagery_stereo
    # resets camera_data.R to identity -- needed for an IMU.T_b_c1 computation downstream.
    left_rectify_R = cam_data_left.R.copy()

    print("=== Loading images from bags ===")
    left_image_data = ImageDataOnDisk.from_ros1_bag(left_bag, f'/{robot}/{left_cam_id}/image_raw')
    right_image_data = ImageDataOnDisk.from_ros1_bag(right_bag, f'/{robot}/{right_cam_id}/image_raw')
    assert left_image_data.encoding == right_image_data.encoding, "Left/Right image encodings must match!"
    assert left_image_data.encoding == ImageData.ImageEncoding.Mono8, "Expected AirMuseum imagery to be Mono8"
    print(f"Left images: {left_image_data.len()}, Right images: {right_image_data.len()}")

    CameraData.align_ImageData_and_CameraData_to_imu_ts([left_image_data], cam_data_left)
    CameraData.align_ImageData_and_CameraData_to_imu_ts([right_image_data], cam_data_right)

    ImageDataOnDisk.crop_to_matched(left_image_data, right_image_data, Decimal('0.01'))
    print(f"After stereo sync: {left_image_data.len()} matched pairs")

    ImageDataOnDisk.undistort_imagery_stereo(left_image_data, right_image_data, cam_data_left, cam_data_right)
    print(f"Post-rectification D_left={cam_data_left.D}, D_right={cam_data_right.D}")

    left_image_data.crop_data(start, end)
    right_image_data.crop_data(start, end)
    print(f"After crop: {left_image_data.len()} images")

    print("=== Loading ground truth ===")
    ground_truth = OdometryData.from_txt(
        input_path / "body_stamped_groundtruth.txt", 'world', 'imu', CoordinateFrame.NONE,
        True, [0, 1, 2, 3, 7, 4, 5, 6])
    ground_truth.redefine_local_axes(ROBOT_TO_GT_FRAME[robot], CoordinateFrame.FLU)
    ground_truth.crop_data(start, end)
    print(f"GT poses: {ground_truth.len()}")

    print("=== Loading IMU from bag ===")
    imu_data = ImuData.from_ros1_bag(imu_bag, f'/{robot}/imu', f'{robot}_imu')
    print(f"IMU samples (raw): {imu_data.len()}")
    imu_data.crop_data(start, end)
    print(f"IMU samples after crop: {imu_data.len()}")

    return AirMuseumRobotData(
        robot=robot, left_cam_id=left_cam_id, right_cam_id=right_cam_id,
        calib_yaml_path=calib_yaml_path, cam_data_left=cam_data_left, cam_data_right=cam_data_right,
        left_rectify_R=left_rectify_R, left_image_data=left_image_data, right_image_data=right_image_data,
        imu_data=imu_data, ground_truth=ground_truth,
    )
