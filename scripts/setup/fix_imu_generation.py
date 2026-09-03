#!/usr/bin/env python3
"""
Fixed IMU generation script with proper frame handling for ORB-SLAM3

Key fixes:
1. Proper gravity handling for AirSim NED frame
2. Output only 7 columns (no world quaternion) for ORB-SLAM3
3. Better debugging output
"""

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, RotationSpline
import sys

# AirSim uses NED (North-East-Down) frame
# In NED: gravity points DOWN (+Z direction)
GRAVITY_NED = np.array([0.0, 0.0, 9.80665], dtype=float)


def load_odom(odom_file: str):
    """Load odometry: t x y z qw qx qy qz"""
    data = []
    with open(odom_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 8:
                try:
                    float(parts[0])  # test if first is numeric
                    data.append([float(x) for x in parts[:8]])
                except:
                    continue

    data = np.array(data)
    ts = data[:, 0]
    pos = data[:, 1:4]
    # Convert qw,qx,qy,qz to qx,qy,qz,qw (SciPy order)
    quat_xyzw = data[:, [5, 6, 7, 4]]

    # Ensure strictly increasing time
    keep = np.concatenate([[True], np.diff(ts) > 0])
    ts, pos, quat_xyzw = ts[keep], pos[keep], quat_xyzw[keep]

    # Normalize quaternions and enforce continuity
    quat_xyzw = quat_xyzw / np.linalg.norm(quat_xyzw, axis=1, keepdims=True)
    for i in range(1, len(quat_xyzw)):
        if np.dot(quat_xyzw[i-1], quat_xyzw[i]) < 0:
            quat_xyzw[i] *= -1

    return ts, pos, quat_xyzw


def compute_imu(ts_odom, pos, quat_xyzw, ts_out):
    """
    Compute IMU measurements from odometry.
    Returns: acc_body (specific force), omega_body (angular velocity)
    """
    # Position spline with clamped boundary conditions
    dt0 = ts_odom[1] - ts_odom[0]
    dt1 = ts_odom[-1] - ts_odom[-2]
    v0 = (pos[1] - pos[0]) / dt0
    v1 = (pos[-1] - pos[-2]) / dt1

    pos_spline = CubicSpline(ts_odom, pos, axis=0, bc_type=((1, v0), (1, v1)))

    # World-frame acceleration
    acc_world = pos_spline(ts_out, 2)

    # Specific force (subtract gravity)
    spec_force_world = acc_world - GRAVITY_NED

    # Rotation spline
    rots_in = Rotation.from_quat(quat_xyzw)
    rot_spline = RotationSpline(ts_odom, rots_in)
    rots_out = rot_spline(ts_out)

    # Transform specific force to body frame
    # Body accel = R^T * (world_accel - gravity)
    acc_body = rots_out.inv().apply(spec_force_world)

    # Angular velocity from rotation differences
    dt = np.diff(ts_out)
    omega_body = np.zeros((len(ts_out), 3))

    for i in range(len(ts_out) - 1):
        if dt[i] > 0:
            delta_rot = rots_out[i].inv() * rots_out[i+1]
            omega_body[i] = delta_rot.as_rotvec() / dt[i]

    omega_body[-1] = omega_body[-2]

    return acc_body, omega_body, rots_out


def save_imu_6dof(ts, acc, omega, out_file):
    """Save IMU in 6-DOF format for ORB-SLAM3"""
    with open(out_file, 'w') as f:
        for t, a, w in zip(ts, acc, omega):
            f.write(f"{t:.6f} {a[0]:.6f} {a[1]:.6f} {a[2]:.6f} "
                   f"{w[0]:.6f} {w[1]:.6f} {w[2]:.6f}\n")


def analyze_motion(ts, acc, omega):
    """Analyze motion characteristics for debugging"""
    acc_mag = np.linalg.norm(acc, axis=1)
    omega_mag = np.linalg.norm(omega, axis=1)

    print(f"\n=== Motion Analysis ===")
    print(f"Duration: {ts[-1] - ts[0]:.2f}s ({len(ts)} samples)")
    print(f"Sample rate: {len(ts)/(ts[-1]-ts[0]):.1f} Hz")
    print(f"\nAcceleration magnitude [m/s²]:")
    print(f"  Mean: {acc_mag.mean():.4f}, Std: {acc_mag.std():.4f}")
    print(f"  Min: {acc_mag.min():.4f}, Max: {acc_mag.max():.4f}")
    print(f"\nAngular velocity magnitude [rad/s]:")
    print(f"  Mean: {omega_mag.mean():.6f}, Std: {omega_mag.std():.6f}")
    print(f"  Min: {omega_mag.min():.6f}, Max: {omega_mag.max():.6f}")

    # Find when motion starts (significant acceleration or rotation)
    motion_threshold_acc = 0.5  # m/s²
    motion_threshold_omega = 0.05  # rad/s

    motion_mask = (acc_mag > motion_threshold_acc) | (omega_mag > motion_threshold_omega)
    if motion_mask.any():
        first_motion_idx = np.argmax(motion_mask)
        first_motion_time = ts[first_motion_idx]
        print(f"\nFirst significant motion at t={first_motion_time:.2f}s (sample {first_motion_idx})")
    else:
        print(f"\nWARNING: No significant motion detected!")

    # Check initial stationary period
    stationary_mask = (acc_mag < 0.1) & (omega_mag < 0.01)
    if stationary_mask[:100].sum() > 50:
        print(f"WARNING: Drone appears stationary for first ~{stationary_mask[:100].sum()} samples")
        print(f"         This will prevent ORB-SLAM3 IMU initialization!")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python fix_imu_generation.py <odom.txt> <output_imu.txt> [--rate 200]")
        sys.exit(1)

    odom_file = sys.argv[1]
    out_file = sys.argv[2]

    imu_rate = None
    if "--rate" in sys.argv:
        idx = sys.argv.index("--rate")
        imu_rate = float(sys.argv[idx + 1])

    print(f"Loading odometry from: {odom_file}")
    ts_odom, pos, quat_xyzw = load_odom(odom_file)
    print(f"Loaded {len(ts_odom)} odometry samples")

    # Generate output timestamps
    if imu_rate:
        t0, t1 = ts_odom[0], ts_odom[-1]
        dt = 1.0 / imu_rate
        ts_out = np.arange(t0, t1 + 0.5*dt, dt)
        print(f"Generating IMU at {imu_rate} Hz ({len(ts_out)} samples)")
    else:
        ts_out = ts_odom
        print(f"Using odometry timestamps ({len(ts_out)} samples)")

    # Compute IMU
    print("Computing IMU measurements...")
    acc_body, omega_body, rots = compute_imu(ts_odom, pos, quat_xyzw, ts_out)

    # Analyze motion
    analyze_motion(ts_out, acc_body, omega_body)

    # Save
    print(f"\nSaving to: {out_file}")
    save_imu_6dof(ts_out, acc_body, omega_body, out_file)
    print("Done!")
