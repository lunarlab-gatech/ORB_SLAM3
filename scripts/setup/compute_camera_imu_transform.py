#!/usr/bin/env python3
"""
Compute IMU.T_b_c1 transformation matrix for ORB-SLAM3

From AirSim settings:
- IMU at body frame origin: (0, 0, 0)
- Camera (stereo_left/depth) at: X=0.35, Y=-0.055, Z=0.2, Pitch=10°

T_b_c1 transforms from camera frame to body (IMU) frame
"""

import numpy as np
from scipy.spatial.transform import Rotation

# Camera position relative to IMU (in body frame)
# AirSim uses NED frame: X=forward, Y=right, Z=down
camera_pos_in_body = np.array([0.35, -0.055, 0.2])  # meters

# Camera orientation relative to body
# Pitch=10° means camera is tilted DOWN by 10° from horizontal
pitch_deg = 10.0
roll_deg = 0.0
yaw_deg = 0.0

# Create rotation matrix: body -> camera
# In AirSim NED: pitch rotation is around Y axis
R_body_to_cam = Rotation.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True).as_matrix()

# For ORB-SLAM3, we need T_b_c1: transformation from camera to body
# T_cam_to_body = inv(T_body_to_cam)
R_cam_to_body = R_body_to_cam.T
t_cam_to_body = -R_cam_to_body @ camera_pos_in_body

# Build 4x4 transformation matrix
T_b_c1 = np.eye(4)
T_b_c1[:3, :3] = R_cam_to_body
T_b_c1[:3, 3] = t_cam_to_body

print("=" * 70)
print("Camera-IMU Extrinsic Calibration for ORB-SLAM3")
print("=" * 70)
print(f"\nCamera position in body frame: {camera_pos_in_body}")
print(f"Camera orientation (RPY): [{roll_deg}, {pitch_deg}, {yaw_deg}] degrees")
print(f"\nTransformation matrix T_b_c1 (camera -> body):")
print(T_b_c1)

print("\n" + "=" * 70)
print("YAML format for Drone1_RGBD_Inertial.yaml:")
print("=" * 70)
print("\nIMU.T_b_c1: !!opencv-matrix")
print("   rows: 4")
print("   cols: 4")
print("   dt: f")
print("   data: [", end="")

# Format as comma-separated list
data_list = T_b_c1.flatten().tolist()
for i, val in enumerate(data_list):
    if i > 0:
        print(", ", end="")
    if i > 0 and i % 4 == 0:
        print("\n          ", end="")
    print(f"{val:.6f}", end="")
print("]")

print("\n" + "=" * 70)

# Verify inverse
T_c1_b = np.linalg.inv(T_b_c1)
print("\nVerification - T_c1_b (body -> camera):")
print("Translation (camera position in body):", T_c1_b[:3, 3])
print("Should match:", camera_pos_in_body)
print("Match:", np.allclose(T_c1_b[:3, 3], camera_pos_in_body))
