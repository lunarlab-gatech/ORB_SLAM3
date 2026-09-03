#!/bin/bash
# Run AirMuseum Scenario5 Drone - Stereo-Inertial

DATASET_PATH="/workspace/datasets/AirMuseum"
OUTPUT_PREFIX="airmuseum_drone_stereo_inertial"

echo "=========================================="
echo "Running AirMuseum Drone - Stereo-Inertial"
echo "=========================================="

cd "$(dirname "$0")"

docker run --rm \
  -v "$(pwd)/src:/workspace/src" \
  -v "$(pwd):/workspace" \
  orbslam3-ubuntu22 bash -c "
cd /workspace &&
./src/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_airmuseum \
  ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ${DATASET_PATH}/Drone_StereoInertial.yaml \
  ${DATASET_PATH}/drone/rgb_stereo_left \
  ${DATASET_PATH}/drone/rgb_stereo_right \
  ${DATASET_PATH}/drone/imu.txt \
  ${OUTPUT_PREFIX}_traj
" 2>&1 | tee ${OUTPUT_PREFIX}.log

echo ""
echo "Comparing trajectory..."
python3 "$(dirname "$0")/../analyze/compare_trajectory_simple.py" \
  datasets/AirMuseum/drone/gt_tum.txt \
  ${OUTPUT_PREFIX}_traj_traj.txt 2>&1 | tee -a ${OUTPUT_PREFIX}.log

cp trajectory_comparison.png ${OUTPUT_PREFIX}_trajectory_comparison.png 2>/dev/null

echo ""
echo "Experiment complete!"
