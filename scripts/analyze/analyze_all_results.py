#!/usr/bin/env python3
"""
Quick analysis of all ORB-SLAM3 runs
"""

import numpy as np
import sys
from pathlib import Path
import glob

def load_tum(filepath):
    """Load TUM format trajectory"""
    try:
        data = np.loadtxt(filepath)
        return data[:, 0], data[:, 1:4], data[:, 4:8]  # ts, pos, quat
    except:
        return None, None, None

def align_and_compute_ate(gt_pos, est_pos):
    """Align using Horn's method and compute ATE"""
    # Center
    gt_mean = np.mean(gt_pos, axis=0)
    est_mean = np.mean(est_pos, axis=0)
    gt_c = gt_pos - gt_mean
    est_c = est_pos - est_mean

    # Scale
    gt_var = np.sum(gt_c ** 2)
    est_var = np.sum(est_c ** 2)
    scale = np.sqrt(gt_var / est_var) if est_var > 0 else 1.0

    # Rotation
    H = est_c.T @ gt_c
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Transform
    t = gt_mean - scale * (R @ est_mean)
    est_aligned = scale * (est_pos @ R.T) + t

    # ATE
    errors = np.linalg.norm(gt_pos - est_aligned, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))

    return rmse, scale, np.mean(errors), np.median(errors)

def main():
    gt_file = Path("/home/sgarimella34/multi-robot-coordination/hercules_datasets/ausenv_roadseq/Drone1/pose_world_frame.txt")

    # Load ground truth
    gt_ts, gt_pos, _ = load_tum(gt_file)
    if gt_pos is None:
        print("ERROR: Cannot load ground truth")
        return

    # Find all Camera Trajectory files
    traj_files = glob.glob("CameraTrajectory*.txt") + ["CameraTrajectory.txt"]
    traj_files = list(set(traj_files))  # Remove duplicates

    results = []

    for traj_file in sorted(traj_files):
        est_ts, est_pos, _ = load_tum(traj_file)
        if est_pos is None or len(est_pos) < 100:
            continue

        # Sync timestamps
        gt_dict = {round(t, 3): i for i, t in enumerate(gt_ts)}
        matched_gt, matched_est = [], []
        for i, t in enumerate(est_ts):
            t_r = round(t, 3)
            if t_r in gt_dict:
                matched_gt.append(gt_pos[gt_dict[t_r]])
                matched_est.append(est_pos[i])

        if len(matched_gt) < 100:
            continue

        matched_gt = np.array(matched_gt)
        matched_est = np.array(matched_est)

        rmse, scale, mean_err, median_err = align_and_compute_ate(matched_gt, matched_est)

        results.append({
            'file': traj_file,
            'rmse': rmse,
            'scale': scale,
            'mean': mean_err,
            'median': median_err,
            'frames': len(matched_gt)
        })

    # Sort by RMSE
    results.sort(key=lambda x: x['rmse'])

    print("\n" + "=" * 80)
    print("TRAJECTORY COMPARISON RESULTS (sorted by RMSE)")
    print("=" * 80)
    print(f"{'File':<40} {'RMSE':>8} {'Scale':>8} {'Mean':>8} {'Median':>8} {'Frames':>8}")
    print("-" * 80)

    for r in results:
        print(f"{r['file']:<40} {r['rmse']:>8.3f} {r['scale']:>8.4f} {r['mean']:>8.3f} {r['median']:>8.3f} {r['frames']:>8}")

    print("=" * 80)

    if results:
        best = results[0]
        print(f"\n🏆 BEST RESULT: {best['file']}")
        print(f"   RMSE: {best['rmse']:.3f}m, Scale: {best['scale']:.4f}x")

        if best['rmse'] < 3.0:
            print(f"\n✅ TARGET ACHIEVED! RMSE < 3m")
        elif best['rmse'] < 5.0:
            print(f"\n⚠️  Good progress! RMSE < 5m (Target: < 3m)")
        else:
            print(f"\n❌ Need more tuning. RMSE = {best['rmse']:.1f}m (Target: < 3m)")

if __name__ == "__main__":
    main()
