#!/bin/bash
# Run City Drone2 with CONSERVATIVE Loop Closure settings
# - More features (4000) for robust matching
# - Higher FAST threshold for stronger features
# - NaN-safe Sophus patch applied

DATASET_PATH="/media/sgarimella34/hercules-collect/raw_data_hercules/city_cslam_ugvuav_test1"
OUTPUT_PREFIX="city_drone2_conservative_lc"

echo "=========================================="
echo "Running City Drone2 - CONSERVATIVE LC"
echo "Features: 4000, FAST: 20/8, Levels: 12"
echo "NaN-safe Sophus patch: ENABLED"
echo "=========================================="

docker run --rm \
  -v "$(pwd)/src:/workspace/src" \
  -v "$(pwd):/workspace" \
  -v "${DATASET_PATH}:/workspace/datasets/city" \
  orbslam3-ubuntu22 bash -c "
cd /workspace &&
./src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim \
  ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /workspace/datasets/city/Drone2_RGBD_ConservativeLC.yaml \
  /workspace/datasets/city/Drone2/rgb_stereo_left \
  /workspace/datasets/city/Drone2/depth \
  ${OUTPUT_PREFIX}_traj
" 2>&1 | tee ${OUTPUT_PREFIX}.log

echo ""
echo "Comparing trajectory..."
python3 "$(dirname "$0")/../analyze/compare_trajectory_simple.py" \
  "${DATASET_PATH}/Drone2/pose_world_frame.txt" \
  ${OUTPUT_PREFIX}_traj_traj.txt 2>&1 | tee -a ${OUTPUT_PREFIX}.log

# Save results
cp trajectory_comparison.png ${OUTPUT_PREFIX}_trajectory_comparison.png 2>/dev/null

echo ""
echo "Experiment complete!"
echo "Results saved to: ${OUTPUT_PREFIX}_*.txt"
