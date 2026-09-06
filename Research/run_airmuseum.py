from airmuseum_loader import AirMuseumDataLoaderSLAM
from generate_airmuseum_config import AirMuseumConfigGenerator
import argparse
import numpy as np
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "lib"))
import orbslam3_python


class AirMuseumRunner:
    """Feeds one AirMuseum robot's data directly into ORB_SLAM3::System (Stereo-Inertial)."""

    REPO_ROOT = Path(__file__).resolve().parent.parent

    @staticmethod
    def run(robot_name: str, dataset_path: str, output_prefix: str, use_viewer: bool = False) -> None:
        """Loads one robot's data, generates its config, and tracks it through ORB-SLAM3.

        Mirrors Examples/Stereo-Inertial/stereo_inertial_euroc.cc's loop: find the first
        IMU sample at/before the first frame, then for each frame collect IMU samples up
        to that frame's timestamp and call track_stereo, finishing with a trajectory save.

        Args:
            robot_name: One of AirMuseumDataLoaderSLAM.ROBOT_LEFT_CAM's keys.
            dataset_path: Root directory for one dataset version (e.g. ".../Scenario5").
            output_prefix: Trajectory file prefix; also passed through as ORB-SLAM3's sequence_name.
            use_viewer: Opens ORB-SLAM3's live Pangolin map/frame viewer, for debugging.
        """

        imu_yaml_path = str(Path(dataset_path).parent / "sensors" / "imu.yaml")
        vocab_path = str(AirMuseumRunner.REPO_ROOT / "Vocabulary" / "ORBvoc.txt")
        config_path = str(AirMuseumRunner.REPO_ROOT / "Examples" / "Stereo-Inertial" / f"AirMuseum_{robot_name}.yaml")
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
        vTimestampsCam = np.array([float(t) for t in data.left_image_data.timestamps], dtype=np.float64)
        vTimestampsImu = np.array([float(t) for t in data.imu_data.timestamps], dtype=np.float64)

        # Format IMU data as needed for ORB-SLAM: columns [ax, ay, az, gx, gy, gz, t], matching
        # IMU::Point's own constructor order (accel, gyro, timestamp).
        imu_all = np.hstack([data.imu_data.lin_acc.astype(np.float64), data.imu_data.ang_vel.astype(np.float64), vTimestampsImu.reshape(-1, 1)])

        first_imu = 0
        while first_imu < nImu and vTimestampsImu[first_imu] <= vTimestampsCam[0]:
            first_imu += 1
        # Clamp at 0: unlike C++'s UB, vTimestampsImu[-1] here would silently wrap to the last sample.
        first_imu = max(first_imu - 1, 0)

        # Create the SLAM system
        SLAM = orbslam3_python.System(vocab_path, config_path, orbslam3_python.Sensor.IMU_STEREO, use_viewer, sequence_name=output_prefix)

        vTimesTrack = np.zeros(nImages, dtype=np.float64)
        for ni in range(nImages):
            imLeft = data.left_image_data.images[ni]
            imRight = data.right_image_data.images[ni]

            tframe = vTimestampsCam[ni]

            # Load imu measurements from previous frame
            vImuMeas = []
            if ni > 0:
                while first_imu < nImu and vTimestampsImu[first_imu] <= vTimestampsCam[ni]:
                    vImuMeas.append(imu_all[first_imu])
                    first_imu += 1
            vImuMeas = np.array(vImuMeas, dtype=np.float64) if vImuMeas else np.empty((0, 7), dtype=np.float64)

            t1 = time.perf_counter()
            SLAM.track_stereo(imLeft, imRight, tframe, vImuMeas)
            t2 = time.perf_counter()

            ttrack = t2 - t1
            vTimesTrack[ni] = ttrack

            # Wait to load the next frame
            T = 0.0
            if ni < nImages - 1:
                T = vTimestampsCam[ni + 1] - tframe
            elif ni > 0:
                T = tframe - vTimestampsCam[ni - 1]

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
        SLAM.save_trajectory_tum(str(output_dir / f"f_{output_prefix}.txt"))
        SLAM.save_keyframe_trajectory_tum(str(output_dir / f"kf_{output_prefix}.txt"))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="drone", choices=AirMuseumDataLoaderSLAM.ROBOT_LEFT_CAM.keys())
    parser.add_argument("--dataset-path", default=str(Path.home() / "data" / "AirMuseum_dataset" / "Scenario5"),
                         help='e.g. .../AirMuseum_dataset/Scenario5')
    parser.add_argument("--output", default=None, help="Trajectory file prefix")
    parser.add_argument("--viewer", action="store_true", help="Open ORB-SLAM3's live Pangolin viewer, for debugging")
    args = parser.parse_args()

    AirMuseumRunner.run(args.robot, args.dataset_path, args.output or f"airmuseum_{args.robot}", args.viewer)
