/**
* Custom RGB-D-Inertial loader for AirSim datasets
* Based on ORB-SLAM3 stereo-inertial examples
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

void LoadImages(const string &strPathRGB, const string &strPathDepth, const string &strPathTimes,
                vector<string> &vstrImageRGB, vector<string> &vstrImageDepth, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

int main(int argc, char **argv)
{
    if(argc < 6)
    {
        cerr << endl << "Usage: ./rgbd_inertial_airsim path_to_vocabulary path_to_settings path_to_rgb path_to_depth path_to_imu (trajectory_file_name)" << endl;
        return 1;
    }

    string file_name;
    if (argc == 7)
    {
        file_name = string(argv[6]);
    }

    // Retrieve paths to images
    vector<string> vstrImageRGB;
    vector<string> vstrImageDepth;
    vector<double> vTimestampsCam;
    vector<cv::Point3f> vAcc, vGyro;
    vector<double> vTimestampsImu;

    string pathRGB = string(argv[3]);
    string pathDepth = string(argv[4]);
    string pathImu = string(argv[5]);

    // Use depth_png16 for 16-bit depth images in millimeters
    if (pathDepth.find("/depth") != string::npos) {
        size_t pos = pathDepth.find("/depth");
        pathDepth = pathDepth.substr(0, pos) + "/depth_png16";
        cout << "Using 16-bit depth from: " << pathDepth << endl;
    }

    // Create timestamps file path from RGB path
    string pathTimes = pathRGB.substr(0, pathRGB.find_last_of("/")) + "/timestamps.txt";

    cout << "Loading images..." << endl;
    LoadImages(pathRGB, pathDepth, pathTimes, vstrImageRGB, vstrImageDepth, vTimestampsCam);
    cout << "Loaded " << vstrImageRGB.size() << " images" << endl;

    cout << "Loading IMU..." << endl;
    LoadIMU(pathImu, vTimestampsImu, vAcc, vGyro);
    cout << "Loaded " << vTimestampsImu.size() << " IMU measurements" << endl;

    int nImages = vstrImageRGB.size();
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
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_RGBD, false, 0, file_name);

    cv::Mat imRGB, imDepth;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    for(int ni=0; ni<nImages; ni++)
    {
        // Read RGB and depth images from file
        imRGB = cv::imread(vstrImageRGB[ni], cv::IMREAD_UNCHANGED);
        imDepth = cv::imread(vstrImageDepth[ni], cv::IMREAD_UNCHANGED);

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load RGB image at: " << vstrImageRGB[ni] << endl;
            return 1;
        }

        if(imDepth.empty())
        {
            cerr << endl << "Failed to load depth image at: " << vstrImageDepth[ni] << endl;
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
        SLAM.TrackRGBD(imRGB, imDepth, tframe, vImuMeas);

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
        // Removed real-time simulation delay for faster processing
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

void LoadImages(const string &strPathRGB, const string &strPathDepth, const string &strPathTimes,
                vector<string> &vstrImageRGB, vector<string> &vstrImageDepth, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(50000);
    vstrImageRGB.reserve(50000);
    vstrImageDepth.reserve(50000);

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimeStamps.push_back(t);
            vstrImageRGB.push_back(strPathRGB + "/" + s + ".png");
            vstrImageDepth.push_back(strPathDepth + "/" + s + ".png");
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
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;

            // Parse space-separated values: timestamp ax ay az gx gy gz (synthetic IMU format)
            istringstream iss(s);
            iss >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];

            vTimeStamps.push_back(data[0]);
            vAcc.push_back(cv::Point3f(data[1], data[2], data[3]));
            vGyro.push_back(cv::Point3f(data[4], data[5], data[6]));
        }
    }
}
