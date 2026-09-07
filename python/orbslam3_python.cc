// pybind11 bindings for ORB_SLAM3::System, just enough to mirror
// Examples/Stereo-Inertial/stereo_inertial_euroc.cc's loop from Python
// (construct System, TrackStereo per frame, Shutdown, save TUM trajectories).

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <opencv2/core/core.hpp>

#include <System.h>
#include <ImuTypes.h>

#include <stdexcept>

namespace py = pybind11;

/**
 * Wraps a (H, W) numpy array as a single-channel cv::Mat.
 *
 * @param arr (H, W) numpy array. Its dtype must already match cv_type's element type
 *     (e.g. uint8 for CV_8UC1, float32 for CV_32FC1) -- each call site's pybind11
 *     parameter type enforces this via forcecast, not this function.
 * @param cv_type OpenCV element type to wrap the data as (e.g. CV_8UC1, CV_32FC1).
 * @return An OpenCV-owned copy of arr's pixel data.
 * @raises std::runtime_error If arr isn't 2D (e.g. an (H, W, C) color image).
 */
static cv::Mat ToMat(py::array arr, int cv_type) {
    py::buffer_info info = arr.request();
    if (info.ndim != 2) {
        throw std::runtime_error("ToMat: expected a (H, W) image, got ndim=" + std::to_string(info.ndim));
    }
    cv::Mat mat(static_cast<int>(info.shape[0]), static_cast<int>(info.shape[1]), cv_type, info.ptr);
    return mat.clone();
}

/**
 * Parses an (N, 7) float64 IMU array into ORB_SLAM3::IMU::Point samples.
 *
 * @param imu_meas (N, 7) float64 array, columns [ax, ay, az, gx, gy, gz, t] --
 *     matching IMU::Point's own constructor order (accel, gyro, timestamp), with N
 *     possibly 0.
 * @return The parsed samples, in the same order.
 */
static std::vector<ORB_SLAM3::IMU::Point> ParseImuMeas(
        py::array_t<double, py::array::c_style | py::array::forcecast> imu_meas) {
    py::buffer_info info = imu_meas.request();
    const double* row = static_cast<const double*>(info.ptr);
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    vImuMeas.reserve(info.shape[0]);
    for (ssize_t i = 0; i < info.shape[0]; ++i, row += 7) {
        vImuMeas.emplace_back(row[0], row[1], row[2], row[3], row[4], row[5], row[6]);
    }
    return vImuMeas;
}

/**
 * Minimal wrapper around ORB_SLAM3::System for Stereo-Inertial and RGB-D use
 * from Python. Only the calls the AirMuseum pipeline needs are exposed:
 * construction, per-frame tracking, shutdown, and TUM trajectory export.
 */
PYBIND11_MODULE(orbslam3_python, m) {
    py::class_<ORB_SLAM3::System>(m, "System")
        .def(py::init<const std::string&, const std::string&, ORB_SLAM3::System::eSensor, bool, int, const std::string&>(),
             py::arg("vocab_file"), py::arg("settings_file"), py::arg("sensor_type"),
             py::arg("use_viewer") = false, py::arg("initial_seq") = 0, py::arg("sequence_name") = std::string())
        .def("track_stereo",
             /**
              * Feeds one rectified stereo frame plus the IMU samples collected since the
              * previous frame into ORB_SLAM3::System::TrackStereo.
              *
              * @param im_left (H, W) uint8 raw (unrectified) left image.
              * @param im_right (H, W) uint8 raw (unrectified) right image.
              * @param timestamp Frame timestamp, in seconds.
              * @param imu_meas (N, 7) float64 array of IMU samples since the previous frame,
              *     columns [ax, ay, az, gx, gy, gz, t] -- matching IMU::Point's own constructor
              *     order (accel, gyro, timestamp), with N possibly 0.
              * @return None; call save_trajectory_tum()/save_keyframe_trajectory_tum() after the
              *     tracking loop to retrieve results.
              */
             [](ORB_SLAM3::System& self, py::array_t<uint8_t, py::array::c_style | py::array::forcecast> im_left,
                py::array_t<uint8_t, py::array::c_style | py::array::forcecast> im_right, double timestamp,
                py::array_t<double, py::array::c_style | py::array::forcecast> imu_meas) {
                 self.TrackStereo(ToMat(im_left, CV_8UC1), ToMat(im_right, CV_8UC1), timestamp, ParseImuMeas(imu_meas));
             },
             py::arg("im_left"), py::arg("im_right"), py::arg("timestamp"), py::arg("imu_meas"))
        .def("track_rgbd",
             /**
              * Feeds one rectified RGB-D frame plus the IMU samples collected since the
              * previous frame into ORB_SLAM3::System::TrackRGBD. Plain RGB-D (as opposed to
              * IMU_RGBD) ignores imu_meas internally, so callers not using IMU_RGBD can just
              * pass an empty array.
              *
              * @param im (H, W) uint8 rectified grayscale image.
              * @param depth (H, W) float32 depth map, in meters, aligned to im's pixel grid.
              * @param timestamp Frame timestamp, in seconds.
              * @param imu_meas (N, 7) float64 array of IMU samples since the previous frame,
              *     columns [ax, ay, az, gx, gy, gz, t] -- matching IMU::Point's own constructor
              *     order (accel, gyro, timestamp), with N possibly 0.
              * @return None; call save_trajectory_tum()/save_keyframe_trajectory_tum() after the
              *     tracking loop to retrieve results.
              */
             [](ORB_SLAM3::System& self, py::array_t<uint8_t, py::array::c_style | py::array::forcecast> im,
                py::array_t<float, py::array::c_style | py::array::forcecast> depth, double timestamp,
                py::array_t<double, py::array::c_style | py::array::forcecast> imu_meas) {
                 self.TrackRGBD(ToMat(im, CV_8UC1), ToMat(depth, CV_32FC1), timestamp, ParseImuMeas(imu_meas));
             },
             py::arg("im"), py::arg("depth"), py::arg("timestamp"), py::arg("imu_meas"))
        .def("get_image_scale", &ORB_SLAM3::System::GetImageScale)
        .def("shutdown", &ORB_SLAM3::System::Shutdown)
        .def("save_trajectory_tum", &ORB_SLAM3::System::SaveTrajectoryTUM, py::arg("filename"))
        .def("save_keyframe_trajectory_tum", &ORB_SLAM3::System::SaveKeyFrameTrajectoryTUM, py::arg("filename"))
        .def("is_lost", &ORB_SLAM3::System::isLost);

    /** Mirrors ORB_SLAM3::System::eSensor. */
    py::enum_<ORB_SLAM3::System::eSensor>(m, "Sensor")
        .value("MONOCULAR", ORB_SLAM3::System::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::STEREO)
        .value("RGBD", ORB_SLAM3::System::RGBD)
        .value("IMU_MONOCULAR", ORB_SLAM3::System::IMU_MONOCULAR)
        .value("IMU_STEREO", ORB_SLAM3::System::IMU_STEREO)
        .value("IMU_RGBD", ORB_SLAM3::System::IMU_RGBD);
}
