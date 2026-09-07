from airmuseum_loader import AirMuseumDataLoaderSLAM
from generate_airmuseum_config import AirMuseumConfigGenerator
import argparse
import cv2
from enum import Enum
import numpy as np
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "lib"))
import orbslam3_python


class AirMuseumRunner:
    """Feeds one AirMuseum robot's data directly into ORB_SLAM3::System (RGB-D or RGB-D-Inertial)."""

    REPO_ROOT = Path(__file__).resolve().parent.parent

    class Mode(Enum):
        """Which ORB_SLAM3::System::eSensor a run tracks with."""
        RGBD = orbslam3_python.Sensor.RGBD
        RGBD_INERTIAL = orbslam3_python.Sensor.IMU_RGBD

    @staticmethod
    def run(robot_name: str, dataset_path: str, output_prefix: str, mode: Mode = Mode.RGBD_INERTIAL, use_viewer: bool = False) -> None:
        """Loads one robot's data, generates its config, and tracks it through ORB-SLAM3.

        Mirrors Examples/RGB-D/rgbd_tum.cc's loop: call track_rgbd per frame, pace to
        the frame's real timestamp gap, finish with a trajectory save. For
        AirMuseumRunner.Mode.RGBD_INERTIAL, the IMU-collection logic mirrors our own
        prior Stereo-Inertial loop (matching Examples/Stereo-Inertial/stereo_inertial_euroc.cc),
        not Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc's IMU handling --
        that file's extra complexity (an IMU callback thread, accel/gyro rate-mismatch
        interpolation) exists only because it streams from live hardware, which doesn't
        apply to us reading pre-recorded, already time-synced data.

        Args:
            robot_name: One of AirMuseumDataLoaderSLAM.ROBOT_LEFT_CAM's keys.
            dataset_path: Root directory for one dataset version (e.g. ".../Scenario5").
            output_prefix: Trajectory file prefix; also passed through as ORB-SLAM3's sequence_name.
            mode: Which ORB_SLAM3 sensor type to track with.
            use_viewer: Opens ORB-SLAM3's live Pangolin map/frame viewer, for debugging.
        """

        use_imu = mode == AirMuseumRunner.Mode.RGBD_INERTIAL

        if robot_name == "drone":
            print("WARNING: drone's IMU is bursty/irregular and known to crash ORB-SLAM3's "
                  "IMU preintegration; prefer OpenVINS for drone for now.")

        imu_yaml_path = str(Path(dataset_path).parent / "sensors" / "imu.yaml")
        vocab_path = str(AirMuseumRunner.REPO_ROOT / "Vocabulary" / "ORBvoc.txt")
        config_subdir = "RGB-D-Inertial" if use_imu else "RGB-D"
        config_path = str(AirMuseumRunner.REPO_ROOT / "Examples" / config_subdir / f"AirMuseum_{robot_name}.yaml")
        output_dir = AirMuseumRunner.REPO_ROOT / "output" / "airmuseum" / robot_name
        output_dir.mkdir(parents=True, exist_ok=True)

        data = AirMuseumDataLoaderSLAM.load_data(dataset_path, [robot_name])[robot_name]
        AirMuseumConfigGenerator.generate_config(data, imu_yaml_path, config_path)

        # Check we have some data
        nImages = data.left_image_data.len()
        nImu = data.imu_data.len()
        if nImages <= 0 or nImu <= 0:
            raise RuntimeError(f"Failed to load images or IMU (images={nImages}, imu={nImu})")

        # Extract timestamps
        vTimestamps = np.array([float(t) for t in data.left_image_data.timestamps], dtype=np.float64)
        vTimestampsImu = np.array([float(t) for t in data.imu_data.timestamps], dtype=np.float64)

        # Format IMU data as needed for ORB-SLAM: columns [ax, ay, az, gx, gy, gz, t], matching
        # IMU::Point's own constructor order (accel, gyro, timestamp).
        imu_all = np.hstack([data.imu_data.lin_acc.astype(np.float64), data.imu_data.ang_vel.astype(np.float64), vTimestampsImu.reshape(-1, 1)])

        first_imu = 0
        while first_imu < nImu and vTimestampsImu[first_imu] <= vTimestamps[0]:
            first_imu += 1
        # Clamp at 0: unlike C++'s UB, vTimestampsImu[-1] here would silently wrap to the last sample.
        first_imu = max(first_imu - 1, 0)

        # Create the SLAM system
        SLAM = orbslam3_python.System(vocab_path, config_path, mode.value, use_viewer, sequence_name=output_prefix)
        imageScale = SLAM.get_image_scale()
        if imageScale != 1.0:
            print(f"WARNING: imageScale={imageScale} != 1.0 -- this resize path is untested against depth_data's alignment/scale assumptions.")

        vTimesTrack = np.zeros(nImages, dtype=np.float64)
        n_processed = nImages
        for ni in range(nImages):
            imRGB = data.left_image_data.images[ni]
            imD = data.depth_data.images[ni]
            tframe = vTimestamps[ni]

            if imageScale != 1.0:
                width = int(imRGB.shape[1] * imageScale)
                height = int(imRGB.shape[0] * imageScale)
                imRGB = cv2.resize(imRGB, (width, height))
                imD = cv2.resize(imD, (width, height))

            # Load imu measurements from previous frame
            vImuMeas = []
            if use_imu and ni > 0:
                while first_imu < nImu and vTimestampsImu[first_imu] <= vTimestamps[ni]:
                    vImuMeas.append(imu_all[first_imu])
                    first_imu += 1
            vImuMeas = np.array(vImuMeas, dtype=np.float64) if vImuMeas else np.empty((0, 7), dtype=np.float64)

            # ORB-SLAM3's IMU preintegration needs at least 2 samples to integrate over;
            # fewer than that (observed at a sequence's true end, once the IMU stream runs
            # out ahead of the camera stream) segfaults it. Stop here.
            if use_imu and ni > 0 and len(vImuMeas) < 2:
                n_processed = ni
                print(f"WARNING: fewer than 2 IMU samples before frame {ni}/{nImages} -- "
                      f"stopping early ({nImages - ni} frames not processed).")
                break

            t1 = time.perf_counter()
            SLAM.track_rgbd(imRGB, imD, tframe, vImuMeas)
            t2 = time.perf_counter()

            ttrack = t2 - t1
            vTimesTrack[ni] = ttrack

            # Wait to load the next frame
            T = 0.0
            if ni < nImages - 1:
                T = vTimestamps[ni + 1] - tframe
            elif ni > 0:
                T = tframe - vTimestamps[ni - 1]

            if ttrack < T:
                time.sleep(T - ttrack)

            if ni % 200 == 0:
                print(f"frame {ni}/{nImages} ({100.0 * ni / nImages:.1f}%)")

        # NOTE: System::Shutdown() (src/System.cc) only calls RequestFinish() on LocalMapping/
        # LoopClosing -- its actual wait-for-threads-to-exit loop is commented out, and even if
        # restored, LocalMapping::Run() (src/LocalMapping.cc) checks CheckFinish() at the bottom
        # of every loop iteration regardless of its keyframe queue depth, so a backlog of
        # unprocessed keyframes can be silently abandoned right here. Not fixed for now (would
        # need restoring that wait loop, which only fixes thread-exit ordering, plus a new
        # exposed "is LocalMapping idle" check polled from here before ever calling shutdown()
        # -- neither implemented, to avoid changing more core ORB-SLAM3 behavior than necessary).
        SLAM.shutdown()

        # Tracking time statistics (only over frames actually processed, see n_processed above)
        vTimesTrack_sorted = np.sort(vTimesTrack[:n_processed])
        totaltime = float(np.sum(vTimesTrack[:n_processed]))
        print("-------\n")
        print(f"median tracking time: {vTimesTrack_sorted[n_processed // 2]}")
        print(f"mean tracking time: {totaltime / n_processed}")

        SLAM.save_trajectory_tum(str(output_dir / f"f_{output_prefix}.txt"))
        SLAM.save_keyframe_trajectory_tum(str(output_dir / f"kf_{output_prefix}.txt"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="robotA", choices=AirMuseumDataLoaderSLAM.ROBOT_LEFT_CAM.keys())
    parser.add_argument("--dataset-path", default=str(Path.home() / "data" / "AirMuseum_dataset" / "Scenario3"),
                         help='e.g. .../AirMuseum_dataset/Scenario5')
    parser.add_argument("--mode", default=AirMuseumRunner.Mode.RGBD_INERTIAL.name.lower(),
                         choices=[m.name.lower() for m in AirMuseumRunner.Mode], help="Which ORB_SLAM3 sensor type to track with.")
    parser.add_argument("--output", default=None, help="Trajectory file prefix")
    parser.add_argument("--viewer", action="store_true", help="Open ORB-SLAM3's live Pangolin viewer, for debugging")
    args = parser.parse_args()

    mode = AirMuseumRunner.Mode[args.mode.upper()]
    AirMuseumRunner.run(args.robot, args.dataset_path, args.output or f"airmuseum_{args.robot}_{args.mode}", mode, args.viewer)
