/**
* RGB-D loader for AirSim datasets (no IMU)
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathRGB, const string &strPathDepth, const string &strPathTimes,
                vector<string> &vstrImageRGB, vector<string> &vstrImageDepth, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        cerr << endl << "Usage: ./rgbd_airsim path_to_vocabulary path_to_settings path_to_rgb path_to_depth (trajectory_file_name)" << endl;
        return 1;
    }

    string file_name;
    if (argc == 6)
    {
        file_name = string(argv[5]);
    }

    // Retrieve paths
    vector<string> vstrImageRGB;
    vector<string> vstrImageDepth;
    vector<double> vTimestamps;

    string pathRGB = string(argv[3]);
    string pathDepth = string(argv[4]);
    // Use depth_png16 for 16-bit depth images in millimeters
    if (pathDepth.find("/depth") != string::npos) {
        size_t pos = pathDepth.find("/depth");
        pathDepth = pathDepth.substr(0, pos) + "/depth_png16";
        cout << "Using 16-bit depth from: " << pathDepth << endl;
    }
    // Check for trimmed timestamps first (excludes calibration motion)
    string basePath = pathRGB.substr(0, pathRGB.find_last_of("/"));
    string pathTimesTrimmed = basePath + "/timestamps_trimmed.txt";
    string pathTimes = basePath + "/timestamps.txt";

    // Use trimmed timestamps if available
    ifstream testFile(pathTimesTrimmed.c_str());
    if (testFile.good()) {
        pathTimes = pathTimesTrimmed;
        cout << "Using trimmed timestamps (no calibration): " << pathTimes << endl;
    }
    testFile.close();

    cout << "Loading images..." << endl;
    LoadImages(pathRGB, pathDepth, pathTimes, vstrImageRGB, vstrImageDepth, vTimestamps);
    cout << "Loaded " << vstrImageRGB.size() << " images" << endl;

    int nImages = vstrImageRGB.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Create SLAM system (with viewer disabled for stability)
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD, false, 0, file_name);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imDepth;
    for(int ni=0; ni<nImages; ni++)
    {
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

        // Read image
        imRGB = cv::imread(vstrImageRGB[ni], cv::IMREAD_UNCHANGED);
        imDepth = cv::imread(vstrImageDepth[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageRGB[ni] << endl;
            return 1;
        }

        if(imDepth.empty())
        {
            cerr << endl << "Failed to load depth at: " << vstrImageDepth[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB, imDepth, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[ni]=ttrack;

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
