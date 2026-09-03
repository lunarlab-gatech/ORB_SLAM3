#!/bin/bash
# Run an RGB-D experiment on a Hercules multi-robot recording (city, aroundshort,
# or citytest2 scenario), with or without loop closure.
#
# Usage: run_experiment.sh --scenario {city|aroundshort|citytest2} --robot ROBOT [--no-lc]
#   city/aroundshort robots: drone1, drone2, husky1, husky2
#   citytest2 robots:        drone2, husky1, husky2   (no drone1 recording)
#
# NOTE: DATA_SUBDIR below (where each scenario's data lives under ~/data) is a
# best-effort guess carried over from the old per-scenario host paths -- verify/fix
# against the real layout.

usage() { echo "Usage: $0 --scenario {city|aroundshort|citytest2} --robot ROBOT [--no-lc]"; exit 1; }

SCENARIO=""
ROBOT=""
LC=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario) SCENARIO="$2"; shift 2 ;;
    --robot) ROBOT="$2"; shift 2 ;;
    --no-lc) LC=0; shift ;;
    *) usage ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

case "$SCENARIO" in
  city)
    DATA_SUBDIR="raw_data_hercules/city_cslam_ugvuav_test1"
    LABEL="City"
    SUFFIX_LC="lc"; SUFFIX_NOLC="nolc"
    case "$ROBOT" in drone1|drone2|husky1|husky2) ;; *) usage ;; esac
    ;;
  aroundshort)
    DATA_SUBDIR="raw_data_hercules/ausenv_aroundshortseq_2ugvuav"
    LABEL="AroundShort"
    SUFFIX_LC="lc"; SUFFIX_NOLC="nolc"
    case "$ROBOT" in drone1|drone2|husky1|husky2) ;; *) usage ;; esac
    ;;
  citytest2)
    DATA_SUBDIR="raw_data_hercules/city_test2"
    LABEL="CityTest2"
    SUFFIX_LC="baseline"; SUFFIX_NOLC="baseline_nolc"
    case "$ROBOT" in drone2|husky1|husky2) ;; *) usage ;; esac
    ;;
  *) usage ;;
esac

ROBOT_CAP="$(tr '[:lower:]' '[:upper:]' <<< "${ROBOT:0:1}")${ROBOT:1}"  # drone1 -> Drone1

if [[ $LC -eq 1 ]]; then
  YAML_SUFFIX=""
  SUFFIX="$SUFFIX_LC"
  LC_LABEL="WITH Loop Closure"
else
  YAML_SUFFIX="_NoLC"
  SUFFIX="$SUFFIX_NOLC"
  LC_LABEL="NO LOOP CLOSURE"
fi

OUTPUT_PREFIX="${SCENARIO}_${ROBOT}_${SUFFIX}"
DATA_ROOT="$HOME/data/${DATA_SUBDIR}"

echo "========================================="
echo "$LABEL $ROBOT_CAP RGB-D - $LC_LABEL"
echo "Loop Closure: $([[ $LC -eq 1 ]] && echo ENABLED || echo DISABLED)"
echo "========================================="

cd "$REPO_ROOT"

./src/ORB_SLAM3/Examples/RGB-D/rgbd_airsim \
  ./src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ${DATA_ROOT}/${ROBOT_CAP}_RGBD_ScaleFocus${YAML_SUFFIX}.yaml \
  ${DATA_ROOT}/${ROBOT_CAP}/rgb_stereo_left \
  ${DATA_ROOT}/${ROBOT_CAP}/depth \
  ${OUTPUT_PREFIX}_traj 2>&1 | tee ${OUTPUT_PREFIX}.log

if [ -f "${OUTPUT_PREFIX}_traj_traj.txt" ]; then
    cp ${OUTPUT_PREFIX}_traj_traj.txt ${OUTPUT_PREFIX}_result.txt
    cp ${OUTPUT_PREFIX}_traj_kf_traj.txt ${OUTPUT_PREFIX}_kf_result.txt
    echo "Comparing trajectory..."
    python3 "$SCRIPT_DIR/../analyze/compare_trajectory_simple.py" \
        "${DATA_ROOT}/${ROBOT_CAP}/pose_world_frame.txt" \
        ${OUTPUT_PREFIX}_result.txt
    mv trajectory_comparison.png ${OUTPUT_PREFIX}_trajectory_comparison.png 2>/dev/null
fi

echo "Experiment complete!"
