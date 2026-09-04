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
 * Wraps a (H, W) uint8 grayscale numpy array as a CV_8UC1 cv::Mat.
 *
 * @param arr (H, W) uint8 grayscale image.
 * @return An OpenCV-owned copy of arr's pixel data.
 * @raises std::runtime_error If arr isn't 2D (e.g. an (H, W, C) color image).
 */
static cv::Mat ToMatMono(py::array_t<uint8_t, py::array::c_style | py::array::forcecast> arr) {
    py::buffer_info info = arr.request();
    if (info.ndim != 2) {
        throw std::runtime_error("ToMatMono: expected a (H, W) uint8 grayscale image, got ndim=" +
                                  std::to_string(info.ndim));
    }
    cv::Mat mat(static_cast<int>(info.shape[0]), static_cast<int>(info.shape[1]), CV_8UC1, info.ptr);
    return mat.clone();
}

/**
 * Minimal wrapper around ORB_SLAM3::System for Stereo-Inertial use from
 * Python. Only the calls the AirMuseum pipeline needs are exposed:
 * construction, per-frame stereo+IMU tracking, shutdown, and TUM trajectory
 * export.
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
              * @param im_left (H, W) uint8 rectified left image.
              * @param im_right (H, W) uint8 rectified right image.
              * @param timestamp Frame timestamp, in seconds.
              * @param imu_meas (N, 7) float64 array of IMU samples since the previous frame,
              *     columns [t, ax, ay, az, gx, gy, gz] -- i.e. robotdataprocess's ImuData.timestamps,
              *     .lin_acc, and .ang_vel hstacked in that order, with N possibly 0.
              * @return None; call save_trajectory_tum()/save_keyframe_trajectory_tum() after the
              *     tracking loop to retrieve results.
              */
             [](ORB_SLAM3::System& self, py::array_t<uint8_t, py::array::c_style | py::array::forcecast> im_left,
                py::array_t<uint8_t, py::array::c_style | py::array::forcecast> im_right, double timestamp,
                py::array_t<double, py::array::c_style | py::array::forcecast> imu_meas) {
                 py::buffer_info info = imu_meas.request();
                 const double* row = static_cast<const double*>(info.ptr);
                 std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
                 vImuMeas.reserve(info.shape[0]);
                 for (ssize_t i = 0; i < info.shape[0]; ++i, row += 7) {
                     vImuMeas.emplace_back(row[1], row[2], row[3], row[4], row[5], row[6], row[0]);
                 }
                 self.TrackStereo(ToMatMono(im_left), ToMatMono(im_right), timestamp, vImuMeas);
             },
             py::arg("im_left"), py::arg("im_right"), py::arg("timestamp"), py::arg("imu_meas"))
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
