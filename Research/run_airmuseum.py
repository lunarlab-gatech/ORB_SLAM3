#!/usr/bin/env python3
"""
Load one AirMuseum robot's stereo+IMU+GT data via robotdataprocess and feed it
directly into ORB_SLAM3::System in memory through the orbslam3_python pybind11
module -- no flat-file round trip (no PNGs/timestamps.txt/imu.txt written to
disk; images are read straight out of the bag via robotdataprocess's lazy
ImageDataOnDisk, rectified in memory, and handed to ORB-SLAM3 frame by frame).

Loading/syncing/rectification lives in airmuseum_loader.py. The tracking loop
below mirrors Examples/Stereo-Inertial/stereo_inertial_airmuseum.cc: find the
first IMU sample at/before the first frame, then for each frame collect IMU
samples up to that frame's timestamp and call TrackStereo, with periodic
backup saves and a final trajectory save.

Usage:
    python3 run_airmuseum.py --robot drone --sensors-dir /path/to/sensors --config /path/to/Drone_StereoInertial.yaml
"""
import argparse
import sys
import time

import numpy as np

from airmuseum_loader import REPO_ROOT, ROBOTS, load_robot_data

sys.path.insert(0, str(REPO_ROOT / "lib"))
try:
    import orbslam3_python
except ImportError as e:
    raise ImportError(
        f"Could not import orbslam3_python ({e}). Build it first: "
        f"cd {REPO_ROOT} && ./build.sh (requires `pip3 install pybind11`)."
    ) from e


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot", required=True, choices=ROBOTS)
    parser.add_argument("--sensors-dir", required=True,
                         help="Directory with each robot's <robot>_cameras_calib.yaml Kalibr calibration.")
    parser.add_argument("--vocab", default=str(REPO_ROOT / "Vocabulary" / "ORBvoc.txt"))
    parser.add_argument("--config", required=True, help="Path to the ORB-SLAM3 Stereo-Inertial yaml config.")
    parser.add_argument("--output", default=None,
                         help="Trajectory file prefix. Defaults to 'airmuseum_<robot>_direct'.")
    args = parser.parse_args()

    robot = args.robot
    config = args.config
    output_prefix = args.output or f"airmuseum_{robot}_direct"

    data = load_robot_data(robot, args.sensors_dir)
    left_image_data, right_image_data, imu_data = data.left_image_data, data.right_image_data, data.imu_data

    n_images = left_image_data.len()
    n_imu = imu_data.len()
    if n_images <= 0 or n_imu <= 0:
        raise RuntimeError(f"Failed to load images or IMU (images={n_images}, imu={n_imu})")

    cam_ts = np.array([float(t) for t in left_image_data.timestamps], dtype=np.float64)
    imu_ts = np.array([float(t) for t in imu_data.timestamps], dtype=np.float64)
    imu_all = np.hstack([
        imu_ts.reshape(-1, 1),
        imu_data.lin_acc.astype(np.float64),
        imu_data.ang_vel.astype(np.float64),
    ])
    empty_imu = np.empty((0, 7), dtype=np.float64)

    # Find the first IMU sample to be considered (last one at/before the first frame),
    # mirroring stereo_inertial_airmuseum.cc's LoadImages/main setup exactly.
    first_imu = 0
    while first_imu < n_imu and imu_ts[first_imu] <= cam_ts[0]:
        first_imu += 1
    first_imu -= 1

    print("-------")
    print(f"Images: {n_images}")
    print(f"IMU: {n_imu}")

    slam = orbslam3_python.System(args.vocab, config, orbslam3_python.Sensor.IMU_STEREO, False, 0, output_prefix)

    times_track = np.zeros(n_images, dtype=np.float64)
    for ni in range(n_images):
        im_left = left_image_data.images[ni]
        im_right = right_image_data.images[ni]
        tframe = cam_ts[ni]

        imu_rows = []
        if ni > 0:
            while first_imu < n_imu and imu_ts[first_imu] <= cam_ts[ni]:
                imu_rows.append(imu_all[first_imu])
                first_imu += 1
        imu_meas = np.array(imu_rows, dtype=np.float64) if imu_rows else empty_imu

        t1 = time.perf_counter()
        slam.track_stereo(im_left, im_right, tframe, imu_meas)
        times_track[ni] = time.perf_counter() - t1

        if ni % 10 == 0 or ni < 5:
            print(f"Processing frame {ni}/{n_images} ({100.0 * ni / n_images:.1f}%)")

        if ni > 0 and ni % 1000 == 0:
            slam.save_trajectory_tum(f"{output_prefix}_backup_frame{ni}_traj.txt")
            slam.save_keyframe_trajectory_tum(f"{output_prefix}_backup_frame{ni}_kf_traj.txt")
            print(f"  [Backup saved at frame {ni}]")

    slam.shutdown()

    sorted_times = np.sort(times_track)
    print("-------\n")
    print(f"median tracking time: {sorted_times[n_images // 2]}")
    print(f"mean tracking time: {sorted_times.mean()}")

    traj_file = f"{output_prefix}_traj.txt"
    kf_file = f"{output_prefix}_kf_traj.txt"
    slam.save_trajectory_tum(traj_file)
    slam.save_keyframe_trajectory_tum(kf_file)
    print(f"Wrote {traj_file}, {kf_file}")


if __name__ == "__main__":
    main()
