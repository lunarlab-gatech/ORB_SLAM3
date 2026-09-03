#!/bin/bash
# Run City Husky1 with FAST MOTION optimized settings WITH LOOP CLOSURE
# - 5000 features for robust tracking at 3m/s
# - Lower FAST thresholds
# - More scale levels

DATASET_PATH="/media/sgarimella34/hercules-collect/raw_data_hercules/city_cslam_ugvuav_test1"
OUTPUT_PREFIX="city_husky1_fastmotion_lc"

echo "=========================================="
echo "Running City Husky1 - FAST MOTION + LC"
echo "Features: 5000, FAST: 10/3, Levels: 12"
echo "=========================================="

docker run --rm \
  -v "$(pwd)/src:/workspace/src" \
  -v "$(pwd):/workspace" \
  -v "${DATASET_PATH}:/workspace/datasets/city" \
  orbslam3-ubuntu22 bash -c "
cd /workspace &&
./src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim \
  ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /workspace/datasets/city/Husky1_RGBD_FastMotion.yaml \
  /workspace/datasets/city/Husky1/rgb_stereo_left \
  /workspace/datasets/city/Husky1/depth \
  ${OUTPUT_PREFIX}_traj
" 2>&1 | tee ${OUTPUT_PREFIX}.log

echo ""
echo "Comparing trajectory..."
python3 "$(dirname "$0")/../analyze/compare_trajectory_simple.py" \
  "${DATASET_PATH}/Husky1/pose_world_frame.txt" \
  ${OUTPUT_PREFIX}_traj_traj.txt 2>&1 | tee -a ${OUTPUT_PREFIX}.log

cp trajectory_comparison.png ${OUTPUT_PREFIX}_trajectory_comparison.png 2>/dev/null

echo ""
echo "Experiment complete!"
