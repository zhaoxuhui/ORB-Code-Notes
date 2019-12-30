/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv) {
    if (argc != 4) {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    // 由于TUM数据集有自身固定的格式，所以这里作者是相当于把路径写死了
    // 这个rgb文件里存放的是每一帧拍摄的时间戳和对应的影像文件路径，如：1305031790.645155 rgb/1305031790.645155.png
    string strFile = string(argv[3]) + "/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();    // 帧的总数

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // 第一个参数是字典文件路径，注意是txt格式，而不是压塑包，否则会读取不了，报错
    // 第二个参数是设置文件路径，就是yaml格式的文件，存放了相机内参以及一些运行的参数等
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    // Vector for tracking time statistics
    // 这个vector用于记录每一帧的耗时
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for (int ni = 0; ni < nImages; ni++) {
        // Read image from file
        // 这里才开始正式读取每一帧影像的内容和时间戳
        im = cv::imread(string(argv[3]) + "/" + vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (im.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

        // 这里的t1、t2是在统计每一帧Tracking的时间，虽然写的比较复杂，但其实没做什么高深的事情
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        // 将每一帧影像内容和时间戳传给系统，这个函数是单目Tracking的核心
        // 每一帧都会调用一次TrackMonocular函数
        // 虽然它会返回一个Tcw(当前帧所对应的camera到world的变换)，但是这里并没有使用
        SLAM.TrackMonocular(im, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        // 根据上面的t1、t2计算时间差
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        // 将每一帧的tracking耗时放到vector中方便后续统计
        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        // 与每一帧的时间戳有关的处理
        double T = 0;
        if (ni < nImages - 1)    // 如果没有到最后一帧，计算两帧之间的时间戳的差值，也即两帧之间的采样间隔
            T = vTimestamps[ni + 1] - tframe;
        else if (ni > 0)   // 最后一帧的采样间隔就是最后一帧时间戳减去上一帧的时间戳
            T = tframe - vTimestamps[ni - 1];

        // 如果说我们tracking的耗时小于图像帧采样的时间间隔，那我们就计算获得这个差值，先休息一下，等等下一帧的到来
        // 如果说tracking的时间已经超过了帧间采样间隔的话，就不能休息了，要抓紧工作啊
        if (ttrack < T)
            usleep((T - ttrack) * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics // 统计整个过程的耗时
    // 先对每帧耗时简单排个序
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    // 再简单求个和，得到总耗时
    for (int ni = 0; ni < nImages; ni++) {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    // 由于上面已经对耗时从小到大排了个序，所以中位数耗时就很容易获得了
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    // 平均耗时也很容易求得
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory   最后的最后，记得要保存成果啊
    // 这里作者直接把保存文件名写死了，会保存在可执行文件目录下，名字叫KeyFrameTrajectory.txt
    // 这里另外需要注意的是，保存的是关键帧的轨迹，而不是所有帧的轨迹，但其实都差不多
    // SLAM对象中有成员函数SaveTrajectoryTUM可以保存所有帧的轨迹，但对单目情况不适用
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps) {
    // 这个函数的用途就是根据rgb文件中的内容读取影像路径和对应时间戳并返回，并没有真正加载影像

    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    // 跳过前三行的原因是ｒｇｂ文件的前三行是注释
    string s0;
    getline(f, s0);
    getline(f, s0);
    getline(f, s0);

    while (!f.eof()) {
        string s;
        getline(f, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            // 新建两个变量分别存放时间戳和影像路径
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
