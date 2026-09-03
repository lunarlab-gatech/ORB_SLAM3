/**
* Custom Stereo-Inertial loader for AirMuseum dataset
* Images loaded via timestamps.txt + timestamp-named PNGs (rectified stereo pairs)
* IMU format: timestamp ax ay az gx gy gz (space-separated)
* Viewer disabled for headless operation
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include <ctime>
#include <sstream>

#include <opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimesLeft, const string &strPathTimesRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

int main(int argc, char **argv)
{
    if(argc < 6)
    {
        cerr << endl << "Usage: ./stereo_inertial_airmuseum path_to_vocabulary path_to_settings path_to_left path_to_right path_to_imu (trajectory_file_name)" << endl;
        return 1;
    }

    string file_name;
    if (argc == 7)
    {
        file_name = string(argv[6]);
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;

    string pathLeft = string(argv[3]);
    string pathRight = string(argv[4]);
    string pathImu = string(argv[5]);

    // Create timestamps file paths from left/right image paths.
    // Left and right images are named by each camera's own timestamp (they
    // differ by up to ~2ms even after sync), so filenames can't be derived
    // from a single shared timestamps file - each side needs its own list,
    // paired by index (both lists are already frame-synced, sorted ascending).
    string pathTimesLeft = pathLeft.substr(0, pathLeft.find_last_of("/")) + "/timestamps.txt";
    string pathTimesRight = pathRight.substr(0, pathRight.find_last_of("/")) + "/timestamps_right.txt";

    cout << "Loading images..." << endl;
    LoadImages(pathLeft, pathRight, pathTimesLeft, pathTimesRight, vstrImageLeft, vstrImageRight, vTimestampsCam);
    cout << "Loaded " << vstrImageLeft.size() << " images" << endl;

    cout << "Loading IMU..." << endl;
    LoadIMU(pathImu, vTimestampsImu, vAcc, vGyro);
    cout << "Loaded " << vTimestampsImu.size() << " IMU measurements" << endl;

    int nImages = vstrImageLeft.size();
    int nImu = vTimestampsImu.size();

    if((nImages<=0)||(nImu<=0))
    {
        cerr << "ERROR: Failed to load images or IMU" << endl;
        return 1;
    }

    // Find first imu to be considered
    int first_imu = 0;
    while(vTimestampsImu[first_imu]<=vTimestampsCam[0])
        first_imu++;
    first_imu--; // first imu measurement to be considered

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Images: " << nImages << endl;
    cout << "IMU: " << nImu << endl;
    cout.precision(17);

    // Create SLAM system (viewer disabled for headless mode)
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO, false, 0, file_name);

    cv::Mat imLeft, imRight;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    for(int ni=0; ni<nImages; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageLeft[ni] << endl;
            return 1;
        }

        if(imRight.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageRight[ni] << endl;
            return 1;
        }

        double tframe = vTimestampsCam[ni];

        // Load IMU measurements from previous frame
        vImuMeas.clear();

        if(ni>0)
        {
            while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
            {
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu].x, vAcc[first_imu].y, vAcc[first_imu].z,
                                                         vGyro[first_imu].x, vGyro[first_imu].y, vGyro[first_imu].z,
                                                         vTimestampsImu[first_imu]));
                first_imu++;
            }
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft, imRight, tframe, vImuMeas);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Progress output every 10 frames
        if(ni % 10 == 0 || ni < 5)
            cout << "Processing frame " << ni << "/" << nImages << " (" << (100.0*ni/nImages) << "%)" << endl;

        // Periodic trajectory saving every 1000 frames to prevent data loss on crash
        if(ni > 0 && ni % 1000 == 0 && !file_name.empty())
        {
            string traj_backup = file_name + "_backup_frame" + to_string(ni) + "_traj.txt";
            string kf_backup = file_name + "_backup_frame" + to_string(ni) + "_kf_traj.txt";
            SLAM.SaveTrajectoryTUM(traj_backup);
            SLAM.SaveKeyFrameTrajectoryTUM(kf_backup);
            cout << "  [Backup saved at frame " << ni << "]" << endl;
        }

        // Process at maximum speed (no sleep)
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save trajectory
    string traj_file = file_name.empty() ? "CameraTrajectory.txt" : file_name + "_traj.txt";
    string kf_file = file_name.empty() ? "KeyFrameTrajectory.txt" : file_name + "_kf_traj.txt";
    SLAM.SaveTrajectoryTUM(traj_file);
    SLAM.SaveKeyFrameTrajectoryTUM(kf_file);

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimesLeft, const string &strPathTimesRight,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    vector<string> vstrTimesRight;
    vstrTimesRight.reserve(50000);
    ifstream fTimesRight;
    fTimesRight.open(strPathTimesRight.c_str());
    while(!fTimesRight.eof())
    {
        string s;
        getline(fTimesRight,s);
        if(!s.empty())
            vstrTimesRight.push_back(s);
    }

    ifstream fTimes;
    fTimes.open(strPathTimesLeft.c_str());
    vTimeStamps.reserve(50000);
    vstrImageLeft.reserve(50000);
    vstrImageRight.reserve(50000);

    size_t idx = 0;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            if(idx >= vstrTimesRight.size())
            {
                cerr << "ERROR: left/right timestamp count mismatch at index " << idx << endl;
                break;
            }
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimeStamps.push_back(t);
            vstrImageLeft.push_back(strPathLeft + "/" + s + ".png");
            vstrImageRight.push_back(strPathRight + "/" + vstrTimesRight[idx] + ".png");
            idx++;
        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(500000);
    vAcc.reserve(500000);
    vGyro.reserve(500000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if(!s.empty())
        {
            double data[7];

            // Parse space-separated values: timestamp ax ay az gx gy gz
            istringstream iss(s);
            iss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];

            vTimeStamps.push_back(data[0]);
            vAcc.push_back(cv::Point3f(data[1], data[2], data[3]));
            vGyro.push_back(cv::Point3f(data[4], data[5], data[6]));
        }
    }
}
