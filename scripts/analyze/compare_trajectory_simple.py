#!/usr/bin/env python3
"""Simple trajectory comparison without scipy dependency"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt

def load_tum_trajectory(filename):
    """Load trajectory in TUM format: timestamp x y z qx qy qz qw"""
    data = np.loadtxt(filename)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    return timestamps, positions

def align_trajectories(est_pos, gt_pos):
    """Align estimated to ground truth using Horn's method (similarity transform)"""
    # Center
    est_center = np.mean(est_pos, axis=0)
    gt_center = np.mean(gt_pos, axis=0)

    est_centered = est_pos - est_center
    gt_centered = gt_pos - gt_center

    # Compute scale
    scale = np.sqrt(np.sum(gt_centered**2) / np.sum(est_centered**2))

    # Compute rotation using SVD
    H = est_centered.T @ gt_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure proper rotation
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Apply transformation
    aligned = scale * (R @ est_centered.T).T + gt_center

    return aligned, scale

def compute_ate(est_pos, gt_pos):
    """Compute Absolute Trajectory Error"""
    errors = np.linalg.norm(est_pos - gt_pos, axis=1)
    return {
        'rmse': np.sqrt(np.mean(errors**2)),
        'mean': np.mean(errors),
        'median': np.median(errors),
        'std': np.std(errors),
        'min': np.min(errors),
        'max': np.max(errors),
        'errors': errors
    }

def main():
    import sys
    if len(sys.argv) != 3:
        print("Usage: python3 compare_trajectory_simple.py <ground_truth.txt> <estimated.txt>")
        sys.exit(1)

    gt_file = sys.argv[1]
    est_file = sys.argv[2]

    print("Loading trajectories...")
    gt_ts, gt_pos = load_tum_trajectory(gt_file)
    est_ts, est_pos = load_tum_trajectory(est_file)

    print(f"Ground truth: {len(gt_ts)} poses")
    print(f"Estimated: {len(est_ts)} poses")

    # Synchronize timestamps
    synced_est, synced_gt = [], []
    for est_t, est_p in zip(est_ts, est_pos):
        idx = np.argmin(np.abs(gt_ts - est_t))
        if np.abs(gt_ts[idx] - est_t) < 0.03:
            synced_est.append(est_p)
            synced_gt.append(gt_pos[idx])

    synced_est = np.array(synced_est)
    synced_gt = np.array(synced_gt)
    print(f"Synchronized: {len(synced_est)} poses")

    # Align
    print("\nAligning trajectories...")
    aligned_est, scale = align_trajectories(synced_est, synced_gt)

    # Compute ATE
    ate = compute_ate(aligned_est, synced_gt)

    print("\n" + "="*60)
    print("ABSOLUTE TRAJECTORY ERROR (ATE)")
    print("="*60)
    print(f"RMSE:   {ate['rmse']:.6f} m")
    print(f"Mean:   {ate['mean']:.6f} m")
    print(f"Median: {ate['median']:.6f} m")
    print(f"Std:    {ate['std']:.6f} m")
    print(f"Min:    {ate['min']:.6f} m")
    print(f"Max:    {ate['max']:.6f} m")
    print(f"\nScale:  {scale:.6f}")
    print("="*60)

    # Plot
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))

    # XY view
    axes[0, 0].plot(synced_gt[:, 0], synced_gt[:, 1], 'g-', label='Ground Truth', linewidth=2)
    axes[0, 0].plot(aligned_est[:, 0], aligned_est[:, 1], 'r--', label='Estimated', linewidth=1.5)
    axes[0, 0].set_xlabel('X (m)')
    axes[0, 0].set_ylabel('Y (m)')
    axes[0, 0].set_title('Top-Down View (X-Y)')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].axis('equal')

    # XZ view
    axes[0, 1].plot(synced_gt[:, 0], synced_gt[:, 2], 'g-', label='Ground Truth', linewidth=2)
    axes[0, 1].plot(aligned_est[:, 0], aligned_est[:, 2], 'r--', label='Estimated', linewidth=1.5)
    axes[0, 1].set_xlabel('X (m)')
    axes[0, 1].set_ylabel('Z (m)')
    axes[0, 1].set_title('Side View (X-Z)')
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    # Error over time
    axes[1, 0].plot(ate['errors'], 'b-', linewidth=1)
    axes[1, 0].axhline(y=ate['rmse'], color='r', linestyle='--', label=f'RMSE: {ate["rmse"]:.4f}m')
    axes[1, 0].set_xlabel('Frame')
    axes[1, 0].set_ylabel('Error (m)')
    axes[1, 0].set_title('Translation Error over Time')
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    # Error histogram
    axes[1, 1].hist(ate['errors'], bins=50, edgecolor='black')
    axes[1, 1].axvline(x=ate['rmse'], color='r', linestyle='--', linewidth=2, label=f'RMSE: {ate["rmse"]:.4f}m')
    axes[1, 1].set_xlabel('Error (m)')
    axes[1, 1].set_ylabel('Frequency')
    axes[1, 1].set_title('Error Distribution')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    output = 'trajectory_comparison.png'
    plt.savefig(output, dpi=300, bbox_inches='tight')
    print(f"\nSaved plot to: {output}")

if __name__ == '__main__':
    main()
