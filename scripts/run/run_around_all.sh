#!/bin/bash
# =========================================================================
# Around Sequence (ausenv_aroundshortseq_2ugvuav) - All 8 experiments
# All robots WITH and WITHOUT loop closure
# Using trimmed timestamps
# =========================================================================
set -e

WORKSPACE="/home/sgarimella34/slam/orbslam3_ws"
DATASET_BASE="/media/sgarimella34/hercules-collect/raw_data_hercules/ausenv_aroundshortseq_2ugvuav"

cd "$WORKSPACE"

run_experiment() {
    local ROBOT=$1        # Drone1, Drone2, Husky1, Husky2
    local LC_MODE=$2      # lc or nolc
    local ROBOT_LOWER=$(echo $ROBOT | tr '[:upper:]' '[:lower:]')

    if [ "$LC_MODE" = "lc" ]; then
        CONFIG="${DATASET_BASE}/${ROBOT}_RGBD_ScaleFocus.yaml"
        LC_LABEL="WITH Loop Closure"
    else
        CONFIG="${DATASET_BASE}/${ROBOT}_RGBD_ScaleFocus_NoLC.yaml"
        LC_LABEL="NO Loop Closure"
    fi

    OUTPUT_PREFIX="around_${ROBOT_LOWER}_${LC_MODE}"
    GT_FILE="${DATASET_BASE}/${ROBOT}/pose_world_frame.txt"

    echo ""
    echo "========================================="
    echo "Around ${ROBOT} RGB-D - ${LC_LABEL}"
    echo "========================================="
    echo "Config: $CONFIG"
    echo "Output: ${OUTPUT_PREFIX}_traj"
    echo ""

    docker run --rm \
      -v "$WORKSPACE/src:/workspace/src" \
      -v "$WORKSPACE:/workspace" \
      -v "/media/sgarimella34/hercules-collect:/workspace/datasets" \
      orbslam3-ubuntu22 bash -c "
    cd /workspace &&
    ./src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim \
      ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
      /workspace/datasets/raw_data_hercules/ausenv_aroundshortseq_2ugvuav/${ROBOT}_RGBD_ScaleFocus$([ "$LC_MODE" = "nolc" ] && echo "_NoLC").yaml \
      /workspace/datasets/raw_data_hercules/ausenv_aroundshortseq_2ugvuav/${ROBOT}/rgb_stereo_left \
      /workspace/datasets/raw_data_hercules/ausenv_aroundshortseq_2ugvuav/${ROBOT}/depth \
      ${OUTPUT_PREFIX}_traj
    " 2>&1 | tee ${OUTPUT_PREFIX}.log

    # Compare against ground truth
    if [ -f "${OUTPUT_PREFIX}_traj_traj.txt" ]; then
        echo ""
        echo "--- Comparing ${OUTPUT_PREFIX} against ground truth ---"
        python3 "$(dirname "$0")/../analyze/compare_trajectory_simple.py" "$GT_FILE" "${OUTPUT_PREFIX}_traj_traj.txt" 2>&1 | tee -a ${OUTPUT_PREFIX}.log
        cp trajectory_comparison.png "${OUTPUT_PREFIX}_trajectory_comparison.png" 2>/dev/null
        echo "Saved plot: ${OUTPUT_PREFIX}_trajectory_comparison.png"
    else
        echo "ERROR: Trajectory file not found for ${OUTPUT_PREFIX}"
    fi

    echo ""
    echo "========================================="
    echo "${OUTPUT_PREFIX} COMPLETE"
    echo "========================================="
    echo ""
}

echo "Starting all 8 Around sequence experiments..."
echo "Using trimmed timestamps for all robots"
echo ""

# Run all experiments
run_experiment Drone1 lc
run_experiment Drone1 nolc
run_experiment Drone2 lc
run_experiment Drone2 nolc
run_experiment Husky1 lc
run_experiment Husky1 nolc
run_experiment Husky2 lc
run_experiment Husky2 nolc

echo ""
echo "========================================="
echo "ALL AROUND SEQUENCE EXPERIMENTS COMPLETE"
echo "========================================="
echo ""
echo "Results summary:"
for robot in drone1 drone2 husky1 husky2; do
    for lc in lc nolc; do
        f="around_${robot}_${lc}.log"
        rmse=$(grep "RMSE:" "$f" 2>/dev/null | tail -1 | awk '{print $2}')
        scale=$(grep "Scale:" "$f" 2>/dev/null | tail -1 | awk '{print $2}')
        echo "  around_${robot}_${lc}: RMSE=${rmse} Scale=${scale}"
    done
done
