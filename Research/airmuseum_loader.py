from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path
from robotdataprocess import CameraData, CoordinateFrame, ImageData, ImageDataOnDisk, ImuData, OdometryData, TransformationData
from typing import Optional


@dataclass
class AirMuseumRobotData:
    cam_data_left: CameraData  # Rectified: K is the undistorted pinhole intrinsics, D is zero.
    left_image_data: ImageDataOnDisk  # Rectified left camera imagery.
    depth_data: ImageDataOnDisk  # Per-frame depth (meters), aligned to left_image_data's pixel grid.
    imu_data: ImuData
    ground_truth: OdometryData
    H_I_to_LO: TransformationData  # ORB-SLAM3's IMU.T_b_c1: raw Kalibr T_cam_imu, inverted. Same (raw, non-FLU) frame as imu_data.

    # ORB-SLAM3's Stereo.b: the rectified stereo baseline. RGB-D still needs this to
    # manufacture a synthetic right-image coordinate per point (ur = u - mbf/depth), so
    # bundle adjustment can use the same well-constrained stereo edges as real stereo.
    stereo_baseline_m: float

class AirMuseumDataLoaderSLAM:
    """Dataloader for the AirMuseum dataset."""

    # Depth beyond this is treated as no measurement.
    MAX_VALID_DEPTH_M: float = 1000.0

    CROP_TIMES: dict[str, dict[str, tuple[Decimal, Optional[Decimal]]]] = {
        "Scenario3": {
            "drone": (Decimal('0.0'), None),
            "robotA": (Decimal('0.0'), None),
            "robotB": (Decimal('0.0'), None),
            "robotC": (Decimal('0.0'), None),
        },
        "Scenario4": {
            "drone": (Decimal('0.0'), None),
            "robotA": (Decimal('0.0'), None),
            "robotB": (Decimal('0.0'), None),
            "robotC": (Decimal('0.0'), None),
        },
        "Scenario5": {
            "drone": (Decimal('0.0'), None),
            "robotA": (Decimal('0.0'), None),
            "robotB": (Decimal('0.0'), None),
            "robotC": (Decimal('0.0'), None),
        },
    }

    ROBOT_LEFT_CAM: dict = {
        'drone': 'cam100',
        'robotA': 'cam101',
        'robotB': 'cam101',
        'robotC': 'cam101',
    }
    CAM_ID_TO_CALIB_NAME: dict = {
        'cam100': 'cam0',
        'cam101': 'cam1'
    }
    CAM_ID_TO_BAG_NAME: dict = {
        'cam100': 'cam100_imu.bag',
        'cam101': 'cam101.bag'
    }
    NAME_TO_FRAME_MAP: dict = {
        "drone": CoordinateFrame.FLU,
        "robotA": CoordinateFrame.UFL,
        "robotB": CoordinateFrame.UFL,
        "robotC": CoordinateFrame.FUR
    }

    @staticmethod
    def load_data(dataset_path: str, robot_names: list[str]) -> dict[str, "AirMuseumRobotData"]:
        """Load the AirMuseum dataset using robotdataprocess, for feeding
        ORB_SLAM3::System (RGB-D) directly via orbslam3_python.

        The precomputed depth was run on imagery resized down from this loader's
        rectified resolution, so it's resized back up to match before use.

        Args:
            dataset_path: Root directory for one dataset version (e.g. ".../Scenario5").
            robot_names: Robots to load data for.

        Returns:
            {robot_name: AirMuseumRobotData}.
        """

        # Get paths and names
        dataset_version: str = Path(dataset_path).name
        dataset_config_path: Path = Path(dataset_path).parent
        base_path: Path = Path(dataset_path) / "data"
        results_path: Path = Path(dataset_path) / "results"

        # Extract the crop times
        crop_times: dict[str, tuple[Decimal, Optional[Decimal]]] = AirMuseumDataLoaderSLAM.CROP_TIMES[dataset_version]

        robot_data: dict[str, AirMuseumRobotData] = {}
        for robot_name in robot_names:

            # Get useful paths
            input_path: Path = base_path / robot_name
            start, end = crop_times[robot_name]

            # Camera constants
            left_cam_id = AirMuseumDataLoaderSLAM.ROBOT_LEFT_CAM[robot_name]
            right_cam_id = 'cam101' if left_cam_id == 'cam100' else 'cam100'
            left_calib_name = AirMuseumDataLoaderSLAM.CAM_ID_TO_CALIB_NAME[left_cam_id]
            right_calib_name = AirMuseumDataLoaderSLAM.CAM_ID_TO_CALIB_NAME[right_cam_id]
            left_bag = input_path / AirMuseumDataLoaderSLAM.CAM_ID_TO_BAG_NAME[left_cam_id]
            right_bag = input_path / AirMuseumDataLoaderSLAM.CAM_ID_TO_BAG_NAME[right_cam_id]

            # ===================================== Get images, depth, and intrinsics =====================================
            # Load stereo intrinsics
            calib_name = robot_name + "_cameras_calib.yaml"
            cam_data_left, cam_data_right = CameraData.from_kalibr_stereo(
                dataset_config_path / 'sensors' / calib_name, left_calib_name, right_calib_name, alpha=0.0)

            # Load the raw stereo pair (needed to compute the rectification below) and the
            # precomputed depth for the left camera.
            left_image_data = ImageDataOnDisk.from_ros1_bag(left_bag, f'/{robot_name}/{left_cam_id}/image_raw')
            right_image_data = ImageDataOnDisk.from_ros1_bag(right_bag, f'/{robot_name}/{right_cam_id}/image_raw')
            depth_data = ImageDataOnDisk.from_npy_files(results_path / "Fast-FoundationStereo" / robot_name / "depth", left_calib_name)
            assert left_image_data.encoding == right_image_data.encoding, "Left/Right image encodings must match!"
            assert left_image_data.encoding == ImageData.ImageEncoding.Mono8, "Expected AirMuseum imagery to be Mono8"

            # Align image data with IMU timestamps
            CameraData.align_ImageData_and_CameraData_to_imu_ts([left_image_data], cam_data_left)
            CameraData.align_ImageData_and_CameraData_to_imu_ts([right_image_data], cam_data_right)

            # Sync the stereo pair, then rectify both to ideal pinhole images. This
            # updates cam_data_left/right's K to the rectified intrinsics and zeroes D.
            ImageDataOnDisk.crop_to_matched(left_image_data, right_image_data, Decimal('0.01'))
            ImageDataOnDisk.undistort_imagery_stereo(left_image_data, right_image_data, cam_data_left, cam_data_right)

            # No matching needed against depth_data: there's always a depth frame for
            # every left image frame. Just resize depth to match the rectified resolution.
            depth_data.resize(cam_data_left.height, cam_data_left.width)
            depth_data.set_above_threshold_to_zero(AirMuseumDataLoaderSLAM.MAX_VALID_DEPTH_M)

            # Crop the defined start/end boundaries
            left_image_data.crop_data(start, end)
            depth_data.crop_data(start, end)

            # NOTE: Depth skips its alignment, synching, and undistorting as that was already done for it.

            # ==================================== Load Transformations =========================================
            # H_LO_to_I: raw Kalibr T_cam_imu for this robot's left/tracking camera,
            # tagged with this robot's native IMU-mounting convention. Inverting it
            # gives ORB-SLAM3's IMU.T_b_c1 directly, and imu_data below is in this same
            # native, non-FLU frame, so the two stay consistent.
            H_LO_to_I = TransformationData.from_kalibr(
                dataset_config_path / 'sensors' / calib_name, left_calib_name, "T_cam_imu",
                AirMuseumDataLoaderSLAM.NAME_TO_FRAME_MAP[robot_name])
            H_I_to_LO = H_LO_to_I.invert()

            # Stereo.b: the along-X baseline in the rectified frame, not the raw
            # extrinsic's full translation (which may carry small Y/Z from real-world
            # mechanical misalignment that rectification exists to remove). cam_data_right.P
            # (unchanged by undistort_imagery_stereo) already encodes this directly.
            stereo_baseline_m = abs(cam_data_right.P[0, 3] / cam_data_right.P[0, 0])

            # ===================================== Load Ground Truth ===========================================
            # GT is relabeled to a common FLU convention across robots purely for
            # evaluation (each robot's GT is recorded w.r.t. a differently-mounted
            # body frame); this has no bearing on H_I_to_LO/imu_data above.
            ground_truth = OdometryData.from_txt(input_path / "body_stamped_groundtruth.txt", 'world', 'imu', CoordinateFrame.NONE,
                                                 True, [0, 1, 2, 3, 7, 4, 5, 6])
            ground_truth.redefine_local_axes(AirMuseumDataLoaderSLAM.NAME_TO_FRAME_MAP[robot_name], CoordinateFrame.FLU)
            ground_truth.crop_data(start, end)

            # ===================================== Load IMU ===========================================
            # Unused by plain RGB-D (only IMU_RGBD consumes it), kept loaded for the
            # eventual switch to RGB-D-Inertial.
            imu_data = ImuData.from_ros1_bag(
                input_path / AirMuseumDataLoaderSLAM.CAM_ID_TO_BAG_NAME['cam100'], f'/{robot_name}/imu', f'{robot_name}_imu')
            imu_data.crop_data(start, end)

            robot_data[robot_name] = AirMuseumRobotData(
                cam_data_left=cam_data_left, left_image_data=left_image_data, depth_data=depth_data,
                imu_data=imu_data, ground_truth=ground_truth, H_I_to_LO=H_I_to_LO, stereo_baseline_m=stereo_baseline_m,
            )
            print(f"Loaded data for {robot_name} from {input_path}...")

        return robot_data
