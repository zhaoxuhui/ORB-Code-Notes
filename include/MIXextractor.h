//
// Created by root on 20-2-14.
//

#ifndef MIXEXTRACTOR_H
#define MIXEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>
#include "iostream"

using namespace std;

namespace ORB_SLAM2 {
    class MIXextractor {
    public:
        MIXextractor(int nfeatures);

        ~MIXextractor() {};

        void operator()(cv::InputArray image, std::vector<cv::KeyPoint> &keypoints, cv::OutputArray descriptors);

        void
        operator()(cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat descriptors,
                   int rows,
                   int cols);


    protected:
        int nfeatures = 1000;

        vector<cv::Mat> get4Blocks(cv::InputArray img);

        void
        getMixKps(cv::InputArray image, int numKps, std::vector<cv::KeyPoint> &keypoints, cv::OutputArray descriptors);
    };
}


#endif //ORB_SLAM2_MIXEXTRACTOR_H
