#!/usr/bin/env python3
"""Gravity-align AirMuseum Fast-FoundationStereo ORB-SLAM3 trajectories to their ground truth.

Computes the rigid (rotation+translation, no scale) alignment transform from the
GT-time-matched subset, then applies that same transform to the FULL original
estimated trajectory so the delivered file keeps every frame ORB-SLAM3 produced,
not just the ones that happened to have a matching GT timestamp.
"""
import numpy as np
from robotdataprocess.data_types.OdometryData import OdometryData, CoordinateFrame
from robotdataprocess.data_types.PathData import PathData
import evo.core.lie_algebra as lie
import evo.core.sync as sync

ROBOTS = ["drone", "robotA", "robotB", "robotC"]

for robot in ROBOTS:
    est_file = f"airmuseum_{robot}_mono_depth_fast_traj_traj.txt"
    gt_file = f"datasets/AirMuseum/{robot}/gt_tum.txt"
    out_file = f"deliverables/AirMuseum_{robot}_ORBSLAM3_trajectory_gravity_aligned.txt"

    est_full = OdometryData.from_tum(est_file, "world", robot, CoordinateFrame.NONE)
    gt = OdometryData.from_tum(gt_file, "world", robot, CoordinateFrame.FLU)
    print(f"=== {robot} ===")
    print(f"est poses (full): {len(est_full.timestamps)}, gt poses: {len(gt.timestamps)}")

    # Compute the alignment transform on the GT-time-matched subset only
    # (same as PathData.align() does internally), so we get the (r_a, t_a)
    # rigid transform to reuse on the full trajectory below.
    gt_traj = gt.to_evo()
    est_traj = est_full.to_evo()
    gt_traj_synced, est_traj_synced = sync.associate_trajectories(gt_traj, est_traj, 0.1)
    print(f"synced (matched) poses used to compute transform: {len(est_traj_synced.timestamps)}")
    r_a, t_a, s = est_traj_synced.align(gt_traj_synced, correct_scale=False, correct_only_scale=False)
    print(f"scale applied: {s} (should be 1.0 -- correct_scale=False, so this is informational only)")

    # Apply the SAME (r_a, t_a) rigid transform to the FULL original trajectory.
    full_traj = est_full.to_evo()
    full_traj.transform(lie.se3(r_a, t_a))
    est_aligned_full = PathData.from_evo(full_traj, est_full.frame_id, est_full.frame, prune_duplicates=False)
    print(f"aligned full trajectory poses: {len(est_aligned_full.timestamps)}")

    # sanity check: vertical (z) extent vs horizontal (x,y) extent -- should
    # be small for z if gravity/up is now correctly isolated to one axis.
    pos = np.array(est_aligned_full.positions, dtype=float)
    ranges = pos.max(axis=0) - pos.min(axis=0)
    print(f"aligned FULL trajectory position range (x,y,z): {ranges}")

    est_aligned_full.to_tum(out_file)
    print(f"wrote {out_file}\n")

print("Done.")
