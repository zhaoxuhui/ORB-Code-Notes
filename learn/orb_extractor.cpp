//
// Created by root on 20-5-9.
//

#include "ORBextractor.h"
#include <iostream>
#include <opencv/cv.hpp>

using namespace std;
using namespace cv;

int main() {
    Mat img, img_gray, img_show;
    vector<cv::KeyPoint> keypoints;
    Mat descriptors;

    img = imread("/root/test.jpg");
    cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    ORB_SLAM2::ORBextractor orb = ORB_SLAM2::ORBextractor(1000, 1.2, 8, 20, 10);
    orb(img_gray, cv::Mat(), keypoints, descriptors);

    img.copyTo(img_show);
    for (size_t i = 0; i < keypoints.size(); ++i) {
        KeyPoint tmp_kp = keypoints[i];
        cout << endl << "------" << i + 1 << "------" << endl;
        cout << "size:" << tmp_kp.size << endl;
        cout << "angle:" << tmp_kp.angle << endl;
        cout << "octave:" << tmp_kp.octave << endl;
        cout << "pt:" << tmp_kp.pt << endl;
        cout << "response:" << tmp_kp.response << endl;
        circle(img_show, keypoints[i].pt, 2, cv::Scalar(0, 255, 255), 2);
    }
    cout << "Total keypoints:" << keypoints.size() << endl;
    cv::imwrite("/root/test_kps.jpg", img_show);
    return 0;
}