#!/usr/bin/env python3
"""
Simple trajectory comparison script
Computes ATE (Absolute Trajectory Error) between estimated and ground truth trajectories
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

def load_tum_trajectory(filename):
    """Load trajectory in TUM format: timestamp x y z qx qy qz qw"""
    data = np.loadtxt(filename)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    quaternions = data[:, 4:8]
    return timestamps, positions, quaternions

def align_trajectories(est_pos, gt_pos):
    """Align estimated trajectory to ground truth using Umeyama algorithm"""
    # Center the point clouds
    est_centered = est_pos - np.mean(est_pos, axis=0)
    gt_centered = gt_pos - np.mean(gt_pos, axis=0)

    # Compute scale
    scale = np.sqrt(np.sum(gt_centered**2) / np.sum(est_centered**2))

    # Compute rotation using SVD
    H = est_centered.T @ gt_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure proper rotation matrix
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Compute translation
    t = np.mean(gt_pos, axis=0) - scale * R @ np.mean(est_pos, axis=0)

    # Apply transformation
    aligned_est = scale * (R @ est_pos.T).T + t

    return aligned_est, scale, R, t

def compute_ate(est_pos, gt_pos):
    """Compute Absolute Trajectory Error"""
    errors = np.linalg.norm(est_pos - gt_pos, axis=1)
    rmse = np.sqrt(np.mean(errors**2))
    mean_error = np.mean(errors)
    median_error = np.median(errors)
    std_error = np.std(errors)
    min_error = np.min(errors)
    max_error = np.max(errors)

    return {
        'rmse': rmse,
        'mean': mean_error,
        'median': median_error,
        'std': std_error,
        'min': min_error,
        'max': max_error,
        'errors': errors
    }

def main():
    import sys
    if len(sys.argv) != 3:
        print("Usage: python3 compare_trajectory.py <ground_truth.txt> <estimated.txt>")
        sys.exit(1)

    gt_file = sys.argv[1]
    est_file = sys.argv[2]

    # Load trajectories
    print("Loading trajectories...")
    gt_ts, gt_pos, gt_quat = load_tum_trajectory(gt_file)
    est_ts, est_pos, est_quat = load_tum_trajectory(est_file)

    # Synchronize timestamps (use nearest neighbor)
    print(f"Ground truth: {len(gt_ts)} poses")
    print(f"Estimated: {len(est_ts)} poses")

    # Find common timestamps
    synced_est_pos = []
    synced_gt_pos = []
    for est_t, est_p in zip(est_ts, est_pos):
        # Find nearest ground truth timestamp
        idx = np.argmin(np.abs(gt_ts - est_t))
        if np.abs(gt_ts[idx] - est_t) < 0.03:  # 30ms threshold
            synced_est_pos.append(est_p)
            synced_gt_pos.append(gt_pos[idx])

    synced_est_pos = np.array(synced_est_pos)
    synced_gt_pos = np.array(synced_gt_pos)

    print(f"Synchronized: {len(synced_est_pos)} poses")

    # Align trajectories
    print("\nAligning trajectories...")
    aligned_est, scale, R, t = align_trajectories(synced_est_pos, synced_gt_pos)

    # Compute ATE
    print("\n" + "="*60)
    print("ABSOLUTE TRAJECTORY ERROR (ATE) - After Alignment")
    print("="*60)

    ate_results = compute_ate(aligned_est, synced_gt_pos)

    print(f"RMSE:   {ate_results['rmse']:.6f} m")
    print(f"Mean:   {ate_results['mean']:.6f} m")
    print(f"Median: {ate_results['median']:.6f} m")
    print(f"Std:    {ate_results['std']:.6f} m")
    print(f"Min:    {ate_results['min']:.6f} m")
    print(f"Max:    {ate_results['max']:.6f} m")
    print(f"\nScale factor: {scale:.6f}")
    print("="*60)

    # Plot trajectories
    print("\nGenerating plots...")
    fig = plt.figure(figsize=(15, 10))

    # 3D trajectory plot
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.plot(synced_gt_pos[:, 0], synced_gt_pos[:, 1], synced_gt_pos[:, 2],
             'g-', label='Ground Truth', linewidth=2)
    ax1.plot(aligned_est[:, 0], aligned_est[:, 1], aligned_est[:, 2],
             'r--', label='Estimated (aligned)', linewidth=1.5)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory Comparison')
    ax1.legend()
    ax1.grid(True)

    # Top-down view (X-Y)
    ax2 = fig.add_subplot(222)
    ax2.plot(synced_gt_pos[:, 0], synced_gt_pos[:, 1], 'g-', label='Ground Truth', linewidth=2)
    ax2.plot(aligned_est[:, 0], aligned_est[:, 1], 'r--', label='Estimated (aligned)', linewidth=1.5)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top-Down View (X-Y)')
    ax2.legend()
    ax2.grid(True)
    ax2.axis('equal')

    # Error over time
    ax3 = fig.add_subplot(223)
    ax3.plot(ate_results['errors'], 'b-', linewidth=1)
    ax3.axhline(y=ate_results['rmse'], color='r', linestyle='--', label=f'RMSE: {ate_results["rmse"]:.4f}m')
    ax3.set_xlabel('Frame')
    ax3.set_ylabel('Error (m)')
    ax3.set_title('Translation Error over Time')
    ax3.legend()
    ax3.grid(True)

    # Error histogram
    ax4 = fig.add_subplot(224)
    ax4.hist(ate_results['errors'], bins=50, edgecolor='black')
    ax4.axvline(x=ate_results['rmse'], color='r', linestyle='--', linewidth=2, label=f'RMSE: {ate_results["rmse"]:.4f}m')
    ax4.set_xlabel('Error (m)')
    ax4.set_ylabel('Frequency')
    ax4.set_title('Error Distribution')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('trajectory_comparison.png', dpi=300, bbox_inches='tight')
    print("Saved plot to: trajectory_comparison.png")

    # Show plot
    plt.show()

if __name__ == '__main__':
    main()
