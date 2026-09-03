#!/bin/bash

echo "========================================="
echo "Drone1 RGB-D Scale Focus - NO LOOP CLOSURE"
echo "========================================="
echo ""

DATASET_DIR="/home/sgarimella34/multi-robot-coordination/hercules_datasets/ausenv_roadseq/Drone1"
CONFIG_FILE="/home/sgarimella34/multi-robot-coordination/hercules_datasets/ausenv_roadseq/Drone1_RGBD_ScaleFocus_NoLC.yaml"
WORKSPACE="/home/sgarimella34/slam/orbslam3_ws"

# Verify dataset
echo "Dataset directory: $DATASET_DIR"
echo "Using timestamps: $(wc -l < $DATASET_DIR/timestamps.txt) frames"
echo ""

# Clean up previous results
cd "$WORKSPACE"
rm -f CameraTrajectory.txt KeyFrameTrajectory.txt
rm -f drone1_nolc_*.txt drone1_nolc_*.log

# Run ORB-SLAM3 without loop closure
echo "Starting ORB-SLAM3 RGB-D (Loop Closure DISABLED)..."
echo "Output: drone1_nolc_scalefocus.log"
echo ""

docker run --rm \
  -v "$WORKSPACE/src:/workspace/src" \
  -v "$WORKSPACE:/workspace" \
  -v "/home/sgarimella34/multi-robot-coordination/hercules_datasets:/workspace/datasets" \
  orbslam3-ubuntu22 bash -c "
cd /workspace &&
./src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim \
  ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /workspace/datasets/ausenv_roadseq/Drone1_RGBD_ScaleFocus_NoLC.yaml \
  /workspace/datasets/ausenv_roadseq/Drone1/rgb_stereo_left \
  /workspace/datasets/ausenv_roadseq/Drone1/depth \
  drone1_nolc_traj
" 2>&1 | tee drone1_nolc_scalefocus.log

# Save results
if [ -f "CameraTrajectory.txt" ]; then
    echo ""
    echo "Saving results..."
    cp CameraTrajectory.txt drone1_nolc_scalefocus_result.txt
    cp KeyFrameTrajectory.txt drone1_nolc_scalefocus_kf_traj.txt

    POSE_COUNT=$(wc -l < CameraTrajectory.txt)
    KF_COUNT=$(wc -l < KeyFrameTrajectory.txt)

    echo "  Camera trajectory: $POSE_COUNT poses"
    echo "  Keyframe trajectory: $KF_COUNT keyframes"

    # Compare against ground truth
    echo ""
    echo "Comparing against ground truth..."
    python3 "$(dirname "$0")/../analyze/compare_trajectory_simple.py" \
        /home/sgarimella34/multi-robot-coordination/hercules_datasets/ausenv_roadseq/Drone1/pose_world_frame.txt \
        drone1_nolc_scalefocus_result.txt

    mv trajectory_comparison.png drone1_nolc_trajectory_comparison.png
fi

echo ""
echo "========================================="
echo "Experiment complete!"
echo "========================================="
