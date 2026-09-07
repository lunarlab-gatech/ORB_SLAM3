from airmuseum_loader import AirMuseumRobotData
import numpy as np
from pathlib import Path
from typing import Union
import yaml


class AirMuseumConfigGenerator:
    """Generates an ORB-SLAM3 Stereo-Inertial YAML config for one AirMuseum robot."""

    @staticmethod
    def _opencv_matrix(key: str, matrix: np.ndarray) -> str:
        """Formats a matrix as a cv::FileStorage "!!opencv-matrix" YAML block."""
        rows, cols = matrix.shape
        data = ", ".join(f"{v:.17g}" for v in matrix.flatten())
        return f"{key}: !!opencv-matrix\n  rows: {rows}\n  cols: {cols}\n  dt: f\n  data: [{data}]\n"

    @staticmethod
    def _average_rate(timestamps) -> float:
        """Overall average sample rate (Hz): (N-1) / time span, not 1/median(diff) --
        some robots' IMUs arrive in bursts (many samples at once, then a pause), which
        would make a per-diff estimate reflect the intra-burst rate instead."""
        ts = np.array([float(t) for t in timestamps], dtype=np.float64)
        return float((len(ts) - 1) / (ts[-1] - ts[0]))

    @staticmethod
    def generate_config(robot_data: AirMuseumRobotData, imu_yaml_path: Union[str, Path], output_path: Union[str, Path]) -> None:
        """Writes an ORB-SLAM3 Stereo-Inertial YAML config to output_path.

        Args:
            robot_data: One robot's loaded data (see AirMuseumDataLoaderSLAM.load_data).
            imu_yaml_path: Path to AirMuseum_dataset/sensors/imu.yaml (acc_n/gyr_n/acc_w/gyr_w).
            output_path: Where to write the generated .yaml config.
        """

        with open(imu_yaml_path, 'r') as f:
            imu_noise = yaml.safe_load(f)

        cl, cr = robot_data.cam_data_left, robot_data.cam_data_right
        fps = AirMuseumConfigGenerator._average_rate(robot_data.left_image_data.timestamps)
        imu_freq = AirMuseumConfigGenerator._average_rate(robot_data.imu_data.timestamps)

        lines = [
            "%YAML:1.0\n",
            'File.version: "1.0"\n',
            'Camera.type: "KannalaBrandt8"\n',

            f"Camera1.fx: {cl.K[0, 0]!s}\n",
            f"Camera1.fy: {cl.K[1, 1]!s}\n",
            f"Camera1.cx: {cl.K[0, 2]!s}\n",
            f"Camera1.cy: {cl.K[1, 2]!s}\n",
            f"Camera1.k1: {cl.D[0]!s}\n",
            f"Camera1.k2: {cl.D[1]!s}\n",
            f"Camera1.k3: {cl.D[2]!s}\n",
            f"Camera1.k4: {cl.D[3]!s}\n",

            f"Camera2.fx: {cr.K[0, 0]!s}\n",
            f"Camera2.fy: {cr.K[1, 1]!s}\n",
            f"Camera2.cx: {cr.K[0, 2]!s}\n",
            f"Camera2.cy: {cr.K[1, 2]!s}\n",
            f"Camera2.k1: {cr.D[0]!s}\n",
            f"Camera2.k2: {cr.D[1]!s}\n",
            f"Camera2.k3: {cr.D[2]!s}\n",
            f"Camera2.k4: {cr.D[3]!s}\n",

            AirMuseumConfigGenerator._opencv_matrix("Stereo.T_c1_c2", robot_data.H_LO_to_RO.as_matrix()),

            # Overlap left as the full width -- conservative (ComputeStereoFishEyeMatches
            # just searches a wider area, doesn't affect correctness), same as TUM-VI.yaml.
            "Camera1.overlappingBegin: 0\n",
            f"Camera1.overlappingEnd: {cl.width - 1}\n",
            "Camera2.overlappingBegin: 0\n",
            f"Camera2.overlappingEnd: {cr.width - 1}\n",

            f"Camera.width: {cl.width}\n",
            f"Camera.height: {cl.height}\n",
            f"Camera.fps: {round(fps)}\n",  # Settings.cc reads this as an int, not a float.
            "Camera.RGB: 0\n", # Ignored as AirMuseum is greyscale
            "Stereo.ThDepth: 40.0\n", # Maybe need to tune
            "loopClosing: 0\n", # Disabled

            AirMuseumConfigGenerator._opencv_matrix("IMU.T_b_c1", robot_data.H_I_to_LO.as_matrix()),

            f"IMU.NoiseGyro: {imu_noise['gyr_n']!r}\n",
            f"IMU.NoiseAcc: {imu_noise['acc_n']!r}\n",
            f"IMU.GyroWalk: {imu_noise['gyr_w']!r}\n",
            f"IMU.AccWalk: {imu_noise['acc_w']!r}\n",
            f"IMU.Frequency: {imu_freq:.1f}\n",

            "ORBextractor.nFeatures: 1200\n",
            "ORBextractor.scaleFactor: 1.2\n",
            "ORBextractor.nLevels: 8\n",
            "ORBextractor.iniThFAST: 20\n",
            "ORBextractor.minThFAST: 7\n",

            # Required by Settings::readViewer regardless of use_viewer (only actually
            # displayed if the caller passes use_viewer=True); values copied from TUM-VI.yaml's defaults.
            "Viewer.KeyFrameSize: 0.05\n",
            "Viewer.KeyFrameLineWidth: 1.0\n",
            "Viewer.GraphLineWidth: 0.9\n",
            "Viewer.PointSize: 2.0\n",
            "Viewer.CameraSize: 0.08\n",
            "Viewer.CameraLineWidth: 3.0\n",
            "Viewer.ViewpointX: 0.0\n",
            "Viewer.ViewpointY: -0.7\n",
            "Viewer.ViewpointZ: -3.5\n",
            "Viewer.ViewpointF: 500.0\n",
        ]

        with open(output_path, 'w') as f:
            f.writelines(lines)
