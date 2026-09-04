/**
* pybind11 wrapper around ORB_SLAM3::System.
*
* Exposes just enough of the C++ API to mimic the Examples/ loader
* loops from Python: construct a System, feed it stereo(+IMU) frames one at a
* time as numpy arrays, and save the resulting trajectory. This lets a
* dataset be loaded once in Python (e.g. via robotdataprocess) and fed
* directly into ORB-SLAM3 in memory, instead of the Examples/*.cc loaders'
* round trip through flat files on disk.
*
* Only Stereo and Stereo-Inertial are wrapped (see
* scripts/extract/extract_airmuseum_data.py); the other sensor modes in
* ORB_SLAM3::System::eSensor aren't exercised by any caller yet, so they're
* left unwrapped rather than guessed at.
*/
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <opencv2/core/core.hpp>

#include <System.h>
#include <ImuTypes.h>

#include <stdexcept>
#include <vector>

namespace py = pybind11;

namespace {

// Wrap a 2D uint8 numpy array (H, W) as a cv::Mat. Copies the pixel data
// (via clone()) so the returned cv::Mat doesn't alias the numpy array's
// buffer -- simpler and safer than relying on Track* only touching the Mat
// synchronously, at the cost of one copy per frame.
cv::Mat ToMat(py::array_t<uint8_t, py::array::c_style | py::array::forcecast> arr) {
    py::buffer_info info = arr.request();
    if (info.ndim != 2) {
        throw std::runtime_error("Expected a 2D (H, W) uint8 grayscale image array");
    }
    cv::Mat mat(static_cast<int>(info.shape[0]), static_cast<int>(info.shape[1]), CV_8UC1, info.ptr);
    return mat.clone();
}

// Build a vector<IMU::Point> from an (N, 7) float64 array: columns are
// [timestamp, ax, ay, az, gx, gy, gz], matching the space-separated txt
// format used elsewhere in this workspace (e.g. imu.txt). N == 0 is fine
// (no IMU measurements since the previous frame).
std::vector<ORB_SLAM3::IMU::Point> ToImuMeas(
        py::array_t<double, py::array::c_style | py::array::forcecast> arr) {
    py::buffer_info info = arr.request();
    std::vector<ORB_SLAM3::IMU::Point> meas;
    if (info.size == 0) {
        return meas;
    }
    if (info.ndim != 2 || info.shape[1] != 7) {
        throw std::runtime_error("Expected an (N, 7) float64 array: [t, ax, ay, az, gx, gy, gz]");
    }
    const auto n = static_cast<size_t>(info.shape[0]);
    const double* data = static_cast<const double*>(info.ptr);
    meas.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const double* row = data + i * 7;
        meas.emplace_back(row[1], row[2], row[3], row[4], row[5], row[6], row[0]);
    }
    return meas;
}

py::array_t<double> EmptyImuMeas() {
    std::vector<py::ssize_t> shape{0, 7};
    return py::array_t<double>(shape);
}

} // namespace

PYBIND11_MODULE(orbslam3_python, m) {
    m.doc() = "pybind11 bindings for ORB_SLAM3::System (Stereo / Stereo-Inertial)";

    py::enum_<ORB_SLAM3::System::eSensor>(m, "Sensor")
        .value("MONOCULAR", ORB_SLAM3::System::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::STEREO)
        .value("RGBD", ORB_SLAM3::System::RGBD)
        .value("IMU_MONOCULAR", ORB_SLAM3::System::IMU_MONOCULAR)
        .value("IMU_STEREO", ORB_SLAM3::System::IMU_STEREO)
        .value("IMU_RGBD", ORB_SLAM3::System::IMU_RGBD);

    py::class_<ORB_SLAM3::System>(m, "System")
        .def(py::init<const std::string&, const std::string&, ORB_SLAM3::System::eSensor, bool, int, const std::string&>(),
             py::arg("voc_file"), py::arg("settings_file"), py::arg("sensor"),
             py::arg("use_viewer") = false, py::arg("init_fr") = 0, py::arg("sequence_name") = std::string(),
             "Construct and start the SLAM system (Local Mapping, Loop Closing[, Viewer] threads).")
        .def("track_stereo",
             [](ORB_SLAM3::System& self,
                py::array_t<uint8_t, py::array::c_style | py::array::forcecast> im_left,
                py::array_t<uint8_t, py::array::c_style | py::array::forcecast> im_right,
                double timestamp,
                py::array_t<double, py::array::c_style | py::array::forcecast> imu_meas) {
                 Sophus::SE3f pose = self.TrackStereo(ToMat(im_left), ToMat(im_right), timestamp, ToImuMeas(imu_meas));
                 return pose.matrix();
             },
             py::arg("im_left"), py::arg("im_right"), py::arg("timestamp"), py::arg("imu_meas") = EmptyImuMeas(),
             "Process one synchronized, rectified stereo(+IMU) frame. imu_meas is an (N, 7) array of "
             "[timestamp, ax, ay, az, gx, gy, gz] rows covering the interval since the previous frame. "
             "Returns the estimated 4x4 camera pose (world-to-camera), as ORB_SLAM3::System::TrackStereo does.")
        .def("shutdown", &ORB_SLAM3::System::Shutdown,
             "Stop all threads. Must be called before Save*Trajectory*.")
        .def("is_shutdown", &ORB_SLAM3::System::isShutDown)
        .def("save_trajectory_tum", &ORB_SLAM3::System::SaveTrajectoryTUM, py::arg("filename"))
        .def("save_keyframe_trajectory_tum", &ORB_SLAM3::System::SaveKeyFrameTrajectoryTUM, py::arg("filename"))
        .def("get_tracking_state", &ORB_SLAM3::System::GetTrackingState);
}
