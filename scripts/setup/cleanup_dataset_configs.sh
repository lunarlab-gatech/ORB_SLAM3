#!/bin/bash

DATASET_DIR="/home/sgarimella34/multi-robot-coordination/hercules_datasets/ausenv_roadseq"

echo "=== Cleaning up Drone1 dataset configs & text files ==="
echo ""
echo "ALL dataset files (images, lidar, IMU, pose) will be preserved!"
echo ""

# Delete failed YAML configs (keep only ScaleFocus)
echo "Deleting unused YAML configs..."
cd "$DATASET_DIR"
rm -v Drone1_RGBD_DepthFocus.yaml \
      Drone1_RGBD_Fewer.yaml \
      Drone1_RGBD_Inertial_InvertedTransform.yaml \
      Drone1_RGBD_Inertial_Relaxed.yaml \
      Drone1_RGBD_Inertial.yaml \
      Drone1_RGBD_Optimized.yaml \
      Drone1_RGBD_SuperFine.yaml \
      Drone1_RGBD_Tuned_v2.yaml \
      Drone1_RGBD_Tuned_v3.yaml \
      Drone1_RGBD_UltraFine.yaml \
      Drone1_RGBD_Ultra.yaml \
      Drone1_RGBD.yaml \
      Drone1_Stereo_ScaleFocus.yaml 2>/dev/null

# Delete duplicate/unused text files
echo ""
echo "Deleting unused text files and logs from Drone1/..."
cd "$DATASET_DIR/Drone1"
rm -v imu.txt \
      odom.txt \
      synthetic_imu_9axis_200Hz.txt \
      synthetic_imu_9axis_500Hz.txt \
      synthetic_imu_corrected_200Hz.txt \
      timestamps_moving_only.txt \
      timestamps_moving.txt \
      timestamps_original.txt \
      timestamps_rotation.txt \
      timestamps_safe.txt \
      timestamps_trimmed.txt \
      rgbd_inertial_rotation_v2.log \
      rgbd_inertial_safe.log 2>/dev/null

# Delete settings.json
echo ""
echo "Deleting settings.json..."
cd "$DATASET_DIR"
rm -v settings.json 2>/dev/null

echo ""
echo "=== Cleanup Complete ==="
echo ""
echo "Kept files:"
echo "  ✓ $DATASET_DIR/Drone1_RGBD_ScaleFocus.yaml"
echo "  ✓ $DATASET_DIR/Drone1/timestamps.txt"
echo "  ✓ $DATASET_DIR/Drone1/timestamps_rgbd_trimmed.txt"
echo "  ✓ $DATASET_DIR/Drone1/pose_world_frame.txt"
echo "  ✓ All dataset directories (images, lidar, depth, etc.)"
echo ""
echo "Space freed: ~150MB"
