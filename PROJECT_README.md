# ORB-SLAM3: Hercules + AirMuseum

Custom ORB-SLAM3 setup for running visual/RGB-D odometry on the Hercules and AirMuseum
multi-robot datasets, built inside Docker with custom dataset loaders (the stock ORB-SLAM3
examples only support EuRoC/TUM/KITTI/RealSense formats).

This repo holds source and configuration only — build artifacts, the Python venv, and raw
datasets are not tracked (see `.gitignore`). Detailed step-by-step setup is in
[`CLAUDE.md`](CLAUDE.md); this file is a quick orientation.

## What's here

```
docker/Dockerfile              # ORB-SLAM3 build image (Ubuntu 22.04 + Pangolin v0.8)
docker/build_image.sh          # Build the image, tagged orbslam3-ubuntu22
docker/run_container.sh        # Create+start the persistent "orbslam3" container
docker/enter_container.sh      # Attach a shell to the running "orbslam3" container
src/ORB_SLAM3/                 # Modified ORB-SLAM3 fork (UZ-SLAMLab/ORB_SLAM3 base)
  CMakeLists.txt                 # + custom executable targets (see below)
  Examples/RGB-D/rgbd_airsim.cc               # Custom RGB-D loader (AirSim/Hercules/AirMuseum format)
  Examples/RGB-D-Inertial/rgbd_inertial_airsim.cc
  Examples/Stereo-Inertial/stereo_inertial_airmuseum.cc
  Thirdparty/Sophus/sophus/so3.hpp            # NaN-warning patch (doesn't crash on bad rotations)
scripts/                       # extract/, convert/, run/, monitor/, analyze/, setup/ (see scripts/MERGE.md)
datasets/AirMuseum/*.yaml      # ORB-SLAM3 config files (camera/IMU calibration, ORB params)
datasets/AirMuseum/sensors/    # Kalibr calibration + IMU noise params for AirMuseum
*.md                           # Setup guide, session handoffs, per-dataset result writeups
```

## Not included (obtain separately)

- **Datasets**: Hercules (`raw_data_hercules/`) and AirMuseum data are large, external, and
  not public — copy them from wherever your team stores them (see `CLAUDE.md` / the
  `*_SESSION_HANDOFF.md` docs for expected directory layout).
- **ORB Vocabulary** (~125MB): `build.sh` expects `src/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz`
  to exist before it runs (it `tar -xf`s it in place). Run
  `scripts/setup/fetch_orb_vocabulary.sh` to download and extract it from the upstream
  ORB_SLAM3 repo before building — `build.sh` will fail partway through otherwise.
- **Python env** for the extraction/comparison scripts: `python3 -m venv venv && pip install
  numpy opencv-python scipy pyyaml matplotlib`, plus [`robotdataprocess`](https://github.com/lunarlab-gatech/robotdataprocess)
  (`develop` branch) for anything touching ROS-bag datasets (AirMuseum).

## Quick start

1. Build the Docker image: `cd docker && ./build_image.sh` (tags it `orbslam3-ubuntu22`).
2. Start the persistent container: `./run_container.sh` (creates+attaches to a container
   named `orbslam3`, with this repo mounted at `~/orb_slam3_ws` and datasets at `~/data`,
   both inside the container).
   From another shell, `./enter_container.sh` attaches an additional session to it.
3. Build ORB-SLAM3 inside it: see `CLAUDE.md` Step 2 for the custom-loader setup and build
   command (`cd ~/orb_slam3_ws/src/ORB_SLAM3 && ./build.sh`).
4. Prepare a dataset (timestamps, depth conversion) and run a `scripts/run/*.sh` script, or
   follow `CLAUDE.md` for the full manual walkthrough.
5. Evaluate with `scripts/analyze/compare_trajectory_simple.py <ground_truth.txt> <estimated.txt>`.

## Known issues

- Inertial modes (`IMU_STEREO`, `IMU_RGBD`, `IMU_MONOCULAR`) share a fragile code path in this
  ORB_SLAM3 fork's `Tracking::PredictStateIMU()` — a null-pointer dereference has been observed
  causing a deterministic segfault on real (non-synthetic) IMU data. RGB-D-only mode has been
  the reliable path for this project; see `ORBSLAM3_SESSION_HANDOFF.md` / `IMU_ISSUES_AND_FIXES.md`.
- RGB-D-only mode has no gravity reference (no IMU in the loop), so the output trajectory's
  world-frame orientation is arbitrary. If you need a gravity-aligned axis, align the full pose
  (position *and* orientation) to a gravity-referenced ground truth rather than relying on
  ORB-SLAM3's own output — see `scripts/setup/gravity_align_airmuseum.py` for the pattern used here.
