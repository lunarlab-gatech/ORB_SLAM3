#!/bin/bash

echo "========================================="
echo "Drone2 RGB-D Scale Focus - NO LOOP CLOSURE"
echo "========================================="
echo ""

DATASET_DIR="/media/sgarimella34/hercules-collect/raw_data_hercules/ausenv_roadseq_2ugvuav/Drone2"
CONFIG_FILE="/media/sgarimella34/hercules-collect/raw_data_hercules/ausenv_roadseq_2ugvuav/Drone2_RGBD_ScaleFocus_NoLC.yaml"
WORKSPACE="/home/sgarimella34/slam/orbslam3_ws"

# Backup current timestamps and use trimmed version
echo "Setting up trimmed timestamps (4.70s to 892.60s)..."
cp "$DATASET_DIR/timestamps.txt" "$DATASET_DIR/timestamps_backup.txt"
cp "$DATASET_DIR/timestamps_drone2_trimmed.txt" "$DATASET_DIR/timestamps.txt"
echo "Using $(wc -l < $DATASET_DIR/timestamps.txt) frames"
echo ""

# Clean up previous results
cd "$WORKSPACE"
rm -f CameraTrajectory.txt KeyFrameTrajectory.txt
rm -f drone2_nolc_*.txt drone2_nolc_*.log

# Run ORB-SLAM3 without loop closure
echo "Starting ORB-SLAM3 RGB-D (Loop Closure DISABLED)..."
echo "Output: drone2_nolc_scalefocus_trimmed.log"
echo ""

docker run --rm \
  -v "$WORKSPACE/src:/workspace/src" \
  -v "$WORKSPACE:/workspace" \
  -v "/media/sgarimella34/hercules-collect:/workspace/datasets" \
  orbslam3-ubuntu22 bash -c "
cd /workspace &&
./src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim \
  ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /workspace/datasets/raw_data_hercules/ausenv_roadseq_2ugvuav/Drone2_RGBD_ScaleFocus_NoLC.yaml \
  /workspace/datasets/raw_data_hercules/ausenv_roadseq_2ugvuav/Drone2/rgb_stereo_left \
  /workspace/datasets/raw_data_hercules/ausenv_roadseq_2ugvuav/Drone2/depth \
  drone2_nolc_traj
" 2>&1 | tee drone2_nolc_scalefocus_trimmed.log

# Save results
if [ -f "CameraTrajectory.txt" ]; then
    echo ""
    echo "Saving results..."
    cp CameraTrajectory.txt drone2_nolc_scalefocus_result.txt
    cp KeyFrameTrajectory.txt drone2_nolc_scalefocus_kf_traj.txt

    POSE_COUNT=$(wc -l < CameraTrajectory.txt)
    KF_COUNT=$(wc -l < KeyFrameTrajectory.txt)

    echo "  Camera trajectory: $POSE_COUNT poses"
    echo "  Keyframe trajectory: $KF_COUNT keyframes"
fi

# Restore original timestamps
echo ""
echo "Restoring original timestamps..."
mv "$DATASET_DIR/timestamps_backup.txt" "$DATASET_DIR/timestamps.txt"

echo ""
echo "========================================="
echo "Experiment complete!"
echo "========================================="
