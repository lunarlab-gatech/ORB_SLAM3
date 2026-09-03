#!/bin/bash
# Run AirMuseum Scenario5 <Robot> - RGB-D (rectified left image + FoundationStereo
# depth, or the resized Fast-FoundationStereo depth with --fast)
#
# Usage: run_airmuseum_mono_depth.sh --robot {drone|robotA|robotB|robotC} [--fast]

usage() { echo "Usage: $0 --robot {drone|robotA|robotB|robotC} [--fast]"; exit 1; }

ROBOT=""
FAST=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot) ROBOT="$2"; shift 2 ;;
    --fast) FAST=1; shift ;;
    *) usage ;;
  esac
done
case "$ROBOT" in
  drone|robotA|robotB|robotC) ;;
  *) usage ;;
esac

ROBOT_CAP="$(tr '[:lower:]' '[:upper:]' <<< "${ROBOT:0:1}")${ROBOT:1}"  # drone -> Drone, robotA -> RobotA

DATASET_PATH="/workspace/datasets/AirMuseum"
if [[ $FAST -eq 1 ]]; then
  DEPTH_DIR="${ROBOT}_fast/depth"
  OUTPUT_PREFIX="airmuseum_${ROBOT}_mono_depth_fast"
  LABEL="RGB-D (Mono-Depth, Fast-FoundationStereo)"
else
  DEPTH_DIR="${ROBOT}/depth"
  OUTPUT_PREFIX="airmuseum_${ROBOT}_mono_depth"
  LABEL="RGB-D (Mono-Depth)"
fi

echo "=========================================="
echo "Running AirMuseum ${ROBOT_CAP} - ${LABEL}"
echo "=========================================="

cd "$(dirname "$0")"

docker run --rm \
  -v "$(pwd)/src:/workspace/src" \
  -v "$(pwd):/workspace" \
  orbslam3-ubuntu22 bash -c "
cd /workspace &&
./src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim \
  ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ${DATASET_PATH}/${ROBOT_CAP}_RGBD.yaml \
  ${DATASET_PATH}/${ROBOT}/rgb_stereo_left \
  ${DATASET_PATH}/${DEPTH_DIR} \
  ${OUTPUT_PREFIX}_traj
" 2>&1 | tee ${OUTPUT_PREFIX}.log

echo ""
echo "Comparing trajectory..."
python3 "$(dirname "$0")/../analyze/compare_trajectory_simple.py" \
  datasets/AirMuseum/${ROBOT}/gt_tum.txt \
  ${OUTPUT_PREFIX}_traj_traj.txt 2>&1 | tee -a ${OUTPUT_PREFIX}.log

cp trajectory_comparison.png ${OUTPUT_PREFIX}_trajectory_comparison.png 2>/dev/null

echo ""
echo "Experiment complete!"
