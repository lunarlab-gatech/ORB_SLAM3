/**
* Custom Stereo loader for AirSim datasets
* Viewer disabled for headless operation
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        cerr << endl << "Usage: ./stereo_airsim path_to_vocabulary path_to_settings path_to_left path_to_right (trajectory_file_name)" << endl;
        return 1;
    }

    string file_name;
    if (argc == 6)
    {
        file_name = string(argv[5]);
    }

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;

    string pathLeft = string(argv[3]);
    string pathRight = string(argv[4]);
    string pathTimes = pathLeft.substr(0, pathLeft.find_last_of("/")) + "/timestamps.txt";

    LoadImages(pathLeft, pathRight, pathTimes, vstrImageLeft, vstrImageRight, vTimestamps);

    int nImages = vstrImageLeft.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    if(vstrImageRight.size() != vstrImageLeft.size())
    {
        cerr << "ERROR: Different number of left and right images" << endl;
        return 1;
    }

    cout << "Loaded " << nImages << " stereo image pairs" << endl;

    // Create SLAM system (viewer disabled for headless mode)
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO, false, 0, file_name);

    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cv::Mat imLeft, imRight;

    for(int ni=0; ni<nImages; ni++)
    {
        imLeft = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED);

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load left image at: " << vstrImageLeft[ni] << endl;
            return 1;
        }

        if(imRight.empty())
        {
            cerr << endl << "Failed to load right image at: " << vstrImageRight[ni] << endl;
            return 1;
        }

        double tframe = vTimestamps[ni];

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        SLAM.TrackStereo(imLeft, imRight, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[ni] = ttrack;

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
    }

    SLAM.Shutdown();

    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
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

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(50000);
    vstrImageLeft.reserve(50000);
    vstrImageRight.reserve(50000);

    while(!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if(!s.empty())
        {
            double t = stod(s);
            vTimeStamps.push_back(t);
            vstrImageLeft.push_back(strPathLeft + "/" + s + ".png");
            vstrImageRight.push_back(strPathRight + "/" + s + ".png");
        }
    }
}
