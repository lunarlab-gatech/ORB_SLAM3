#!/bin/bash
# Run City Husky2 with BASELINE config (same as Drone1) - TRIMMED (no calibration)
# Time range: 48.35s - 201.15s
# Config: 3000 features, scaleFactor=1.1, nLevels=10

DATASET_PATH="/media/sgarimella34/hercules-collect/raw_data_hercules/city_cslam_ugvuav_test1"
OUTPUT_PREFIX="city_husky2_baseline_trimmed"

echo "=========================================="
echo "Running City Husky2 - BASELINE TRIMMED"
echo "Time: 48.35s - 201.15s (no calibration)"
echo "Features: 3000, Scale: 1.1, Levels: 10"
echo "=========================================="

docker run --rm \
  -v "$(pwd)/src:/workspace/src" \
  -v "$(pwd):/workspace" \
  -v "${DATASET_PATH}:/workspace/datasets/city" \
  orbslam3-ubuntu22 bash -c "
cd /workspace &&
./src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim \
  ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /workspace/datasets/city/Husky2_RGBD_ScaleFocus.yaml \
  /workspace/datasets/city/Husky2/rgb_stereo_left \
  /workspace/datasets/city/Husky2/depth \
  ${OUTPUT_PREFIX}_traj
" 2>&1 | tee ${OUTPUT_PREFIX}.log

echo ""
echo "Comparing trajectory..."
python3 "$(dirname "$0")/../analyze/compare_trajectory_simple.py" \
  "${DATASET_PATH}/Husky2/pose_world_frame.txt" \
  ${OUTPUT_PREFIX}_traj_traj.txt 2>&1 | tee -a ${OUTPUT_PREFIX}.log

cp trajectory_comparison.png ${OUTPUT_PREFIX}_trajectory_comparison.png 2>/dev/null

echo ""
echo "Experiment complete!"
