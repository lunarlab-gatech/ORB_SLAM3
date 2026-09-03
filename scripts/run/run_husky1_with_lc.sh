#!/bin/bash

echo "========================================="
echo "Husky1 RGB-D Scale Focus - WITH Loop Closure"
echo "========================================="
echo ""

DATASET_DIR="/media/sgarimella34/hercules-collect/raw_data_hercules/ausenv_roadseq_2ugvuav/Husky1"
CONFIG_FILE="/media/sgarimella34/hercules-collect/raw_data_hercules/ausenv_roadseq_2ugvuav/Husky1_RGBD_ScaleFocus.yaml"
WORKSPACE="/home/sgarimella34/slam/orbslam3_ws"

# Verify dataset
echo "Dataset directory: $DATASET_DIR"
echo "Using timestamps: $(wc -l < $DATASET_DIR/timestamps.txt) frames (1.50s to 1125.00s)"
echo ""

# Clean up previous results
cd "$WORKSPACE"
rm -f CameraTrajectory.txt KeyFrameTrajectory.txt
rm -f husky1_lc_*.txt husky1_lc_*.log

# Run ORB-SLAM3 WITH loop closure
echo "Starting ORB-SLAM3 RGB-D (Loop Closure ENABLED)..."
echo "Output: husky1_lc_scalefocus.log"
echo ""

docker run --rm \
  -v "$WORKSPACE/src:/workspace/src" \
  -v "$WORKSPACE:/workspace" \
  -v "/media/sgarimella34/hercules-collect:/workspace/datasets" \
  orbslam3-ubuntu22 bash -c "
cd /workspace &&
./src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim \
  ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /workspace/datasets/raw_data_hercules/ausenv_roadseq_2ugvuav/Husky1_RGBD_ScaleFocus.yaml \
  /workspace/datasets/raw_data_hercules/ausenv_roadseq_2ugvuav/Husky1/rgb_stereo_left \
  /workspace/datasets/raw_data_hercules/ausenv_roadseq_2ugvuav/Husky1/depth \
  husky1_lc_traj
" 2>&1 | tee husky1_lc_scalefocus.log

# Save results
if [ -f "CameraTrajectory.txt" ]; then
    echo ""
    echo "Saving results..."
    cp CameraTrajectory.txt husky1_lc_scalefocus_result.txt
    cp KeyFrameTrajectory.txt husky1_lc_scalefocus_kf_traj.txt

    POSE_COUNT=$(wc -l < CameraTrajectory.txt)
    KF_COUNT=$(wc -l < KeyFrameTrajectory.txt)

    echo "  Camera trajectory: $POSE_COUNT poses"
    echo "  Keyframe trajectory: $KF_COUNT keyframes"

    # Compare against ground truth
    echo ""
    echo "Comparing against ground truth..."
    python3 "$(dirname "$0")/../analyze/compare_trajectory_simple.py" \
        /media/sgarimella34/hercules-collect/raw_data_hercules/ausenv_roadseq_2ugvuav/Husky1/pose_world_frame.txt \
        husky1_lc_scalefocus_result.txt

    mv trajectory_comparison.png husky1_lc_trajectory_comparison.png
fi

echo ""
echo "========================================="
echo "Experiment complete!"
echo "========================================="
