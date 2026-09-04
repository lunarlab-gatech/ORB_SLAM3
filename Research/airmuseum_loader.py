from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path
from robotdataprocess import CameraData, CoordinateFrame, ImageData, ImageDataOnDisk, ImuData, OdometryData, TransformationData
from typing import Optional


@dataclass
class AirMuseumRobotData:
    cam_data_left: CameraData
    cam_data_right: CameraData
    left_image_data: ImageDataOnDisk
    right_image_data: ImageDataOnDisk
    imu_data: ImuData
    ground_truth: OdometryData
    H_I_to_LO: TransformationData  # ORB-SLAM3's IMU.T_b_c1: raw Kalibr T_cam_imu, inverted. Same (raw, non-FLU) frame as imu_data.
    H_RO_to_LO: TransformationData  # ORB-SLAM3's Stereo.T_c1_c2 (raw): p_RO = M @ p_LO, i.e. T_right_left -- ORB-SLAM3's "T_c1_c2" name is backwards from that (c1=left, c2=right).

class AirMuseumDataLoaderSLAM:
    """Dataloader for the AirMuseum dataset."""

    CROP_TIMES: dict[str, dict[str, tuple[Decimal, Optional[Decimal]]]] = {
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
        ORB_SLAM3::System (Stereo-Inertial) directly via orbslam3_python.

        No depth here: results/Fast-FoundationStereo's depth maps were computed
        on rectified imagery, but ORB-SLAM3 has no rectification path for this
        calibration's KannalaBrandt8/fisheye model (see the image-loading
        comment below) and no way to give RGB-D a depth-specific calibration
        distinct from Camera1 -- so that depth wouldn't correspond to the raw
        images this loads. A Mono-Depth fallback would need its own separate,
        rectification-aware loading path.

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
            left_bag = input_path / AirMuseumDataLoaderSLAM.CAM_ID_TO_BAG_NAME[left_cam_id]
            right_bag = input_path / AirMuseumDataLoaderSLAM.CAM_ID_TO_BAG_NAME[right_cam_id]

            # ===================================== Get images and intrinsics & stereo sync =====================================
            # Load stereo intrinstics
            calib_name = robot_name + "_cameras_calib.yaml"
            cam_data_left, cam_data_right = CameraData.from_kalibr_stereo(
                dataset_config_path / 'sensors' / calib_name,
                AirMuseumDataLoaderSLAM.CAM_ID_TO_CALIB_NAME[left_cam_id],
                AirMuseumDataLoaderSLAM.CAM_ID_TO_CALIB_NAME[right_cam_id], alpha=0.0)

            # Load the Camera images (raw, unrectified). cam_data_left/right's K/D stay
            # the raw Kalibr calibration too. ORB-SLAM3's KannalaBrandt8 camera model
            # (this calibration's fisheye/equidistant distortion) does its own
            # per-keypoint analytic undistortion and its own raw-image stereo matching
            # (ComputeStereoFishEyeMatches); unlike its PinHole model, it has no
            # internal rectification path, so it expects raw, unrectified images.
            left_image_data = ImageDataOnDisk.from_ros1_bag(left_bag, f'/{robot_name}/{left_cam_id}/image_raw')
            right_image_data = ImageDataOnDisk.from_ros1_bag(right_bag, f'/{robot_name}/{right_cam_id}/image_raw')
            assert left_image_data.encoding == right_image_data.encoding, "Left/Right image encodings must match!"
            assert left_image_data.encoding == ImageData.ImageEncoding.Mono8, "Expected AirMuseum imagery to be Mono8"

            # Align timestamps with the IMU's timestamps
            CameraData.align_ImageData_and_CameraData_to_imu_ts([left_image_data], cam_data_left)
            CameraData.align_ImageData_and_CameraData_to_imu_ts([right_image_data], cam_data_right)

            # Get only synced images
            ImageDataOnDisk.crop_to_matched(left_image_data, right_image_data, Decimal('0.01'))

            # Crop the defined start/end boundaries
            left_image_data.crop_data(start, end)
            right_image_data.crop_data(start, end)

            # ==================================== Load Transformations =========================================
            # H_LO_to_I: raw Kalibr T_cam_imu for this robot's left/tracking camera,
            # tagged with this robot's native IMU-mounting convention. Inverting it
            # gives ORB-SLAM3's IMU.T_b_c1 directly, and imu_data below is in this same
            # native, non-FLU frame, so the two stay consistent.
            H_LO_to_I = TransformationData.from_kalibr(
                dataset_config_path / 'sensors' / calib_name,
                AirMuseumDataLoaderSLAM.CAM_ID_TO_CALIB_NAME[left_cam_id], "T_cam_imu",
                AirMuseumDataLoaderSLAM.NAME_TO_FRAME_MAP[robot_name])
            H_I_to_LO = H_LO_to_I.invert()

            # H_RO_to_LO: ORB-SLAM3's Stereo.T_c1_c2 (see AirMuseumRobotData.H_RO_to_LO).
            # from_kalibr_stereo() doesn't expose this (it only keeps K/D/R/P), so it's
            # read directly here: Kalibr's T_cn_cnm1 always lives under "cam1" and gives
            # cam1's pose w.r.t. cam0, i.e. p_cam1 = T_cn_cnm1 @ p_cam0 -- already
            # H_RO_to_LO if cam1 is our right camera, or its inverse if cam1 is our left.
            H_CN_to_CNM1 = TransformationData.from_kalibr(
                dataset_config_path / 'sensors' / calib_name, 'cam1', "T_cn_cnm1", CoordinateFrame.NONE)
            if AirMuseumDataLoaderSLAM.CAM_ID_TO_CALIB_NAME[right_cam_id] == 'cam0':
                H_RO_to_LO = H_CN_to_CNM1.invert()
            else:
                H_RO_to_LO = H_CN_to_CNM1

            # ===================================== Load Ground Truth ===========================================
            # GT is relabeled to a common FLU convention across robots purely for
            # evaluation (each robot's GT is recorded w.r.t. a differently-mounted
            # body frame); this has no bearing on H_I_to_LO/imu_data above.
            ground_truth = OdometryData.from_txt(input_path / "body_stamped_groundtruth.txt", 'world', 'imu', CoordinateFrame.NONE,
                                                 True, [0, 1, 2, 3, 7, 4, 5, 6])
            ground_truth.redefine_local_axes(AirMuseumDataLoaderSLAM.NAME_TO_FRAME_MAP[robot_name], CoordinateFrame.FLU)
            ground_truth.crop_data(start, end)

            # ===================================== Load IMU ===========================================
            imu_data = ImuData.from_ros1_bag(
                input_path / AirMuseumDataLoaderSLAM.CAM_ID_TO_BAG_NAME['cam100'], f'/{robot_name}/imu', f'{robot_name}_imu')
            imu_data.crop_data(start, end)

            robot_data[robot_name] = AirMuseumRobotData(
                cam_data_left=cam_data_left, cam_data_right=cam_data_right,
                left_image_data=left_image_data, right_image_data=right_image_data,
                imu_data=imu_data, ground_truth=ground_truth, H_I_to_LO=H_I_to_LO, H_RO_to_LO=H_RO_to_LO,
            )
            print(f"Loaded data for {robot_name} from {input_path}...")

        return robot_data
