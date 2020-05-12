//
// Created by root on 20-2-14.
//

#include <opencv2/imgcodecs.hpp>
#include <include/MIXextractor.h>
#include <opencv/cv.hpp>
#include "ORBextractor.h"
#include "iostream"

using namespace cv;
using namespace std;

void bfMatch(vector<cv::KeyPoint> kp1, cv::Mat des1,
             vector<cv::KeyPoint> kp2, cv::Mat des2,
             vector<cv::KeyPoint> &good_kps1,
             vector<cv::KeyPoint> &good_kps2,
             double disTh = 15.) {
    cv::Ptr<cv::DescriptorMatcher> orb_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    vector<cv::DMatch> tmp_match;
    vector<cv::DMatch> matches;
    orb_matcher->match(des1, des2, tmp_match);
    double min_dis = 10000, max_dis = 0;
    for (int i = 0; i < des1.rows; ++i) {
        double dist = tmp_match[i].distance;
        if (dist < min_dis) min_dis = dist;
        if (dist > max_dis) max_dis = dist;
    }

    for (int j = 0; j < des1.rows; ++j) {
        if (tmp_match[j].distance < max(2 * min_dis, disTh)) {
            matches.push_back(tmp_match[j]);
        }
    }

    cout << "keypoints1-> " << kp1.size() << " " << "keypoints2-> " << kp2.size() << endl;
    cout << "matches:" << matches.size() << endl;

    for (int k = 0; k < matches.size(); ++k) {
        good_kps1.push_back(kp1[matches[k].queryIdx]);
        good_kps2.push_back(kp2[matches[k].trainIdx]);
    }
}

void allMethod(cv::Mat img, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors, string path) {
    ORB_SLAM2::MIXextractor ext2 = ORB_SLAM2::MIXextractor(10000);
    ext2.operator()(img, keyPoints, descriptors);
    cout << keyPoints.size() << endl;
    Mat img_2;
    img.copyTo(img_2);
    for (int i = 0; i < keyPoints.size(); ++i) {
        circle(img_2, keyPoints[i].pt, 2, cv::Scalar(0, 255, 255), 2);
    }
    cv::imwrite(path, img_2);
}

void blockMethod(cv::Mat img, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors, string path) {
    ORB_SLAM2::MIXextractor ext1 = ORB_SLAM2::MIXextractor(10000);
    ext1.operator()(img, keyPoints, descriptors, 3, 3);
    cout << keyPoints.size() << endl;
    Mat img_1;
    img.copyTo(img_1);
    for (int i = 0; i < keyPoints.size(); ++i) {
        circle(img_1, keyPoints[i].pt, 2, cv::Scalar(0, 255, 255), 2);
    }
    cv::imwrite(path, img_1);
}

void orbMethod(cv::Mat img, std::vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors, string path) {
    ORB_SLAM2::ORBextractor orb = ORB_SLAM2::ORBextractor(10000, 1, 1, 20, 10);
    Mat img_gray;
    cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    orb.operator()(img_gray, cv::Mat(), keyPoints, descriptors);
    cout << keyPoints.size() << endl;
    Mat img_3;
    img.copyTo(img_3);
    for (int i = 0; i < keyPoints.size(); ++i) {
        circle(img_3, keyPoints[i].pt, 2, cv::Scalar(0, 255, 255), 2);
    }
    cv::imwrite(path, img_3);
}

int main() {
//    Mat img1 = cv::imread("/root/satellite2020/VAZ1_201707091924_002_0033_L1A.tif");
//    Mat img2 = cv::imread("/root/satellite2020/VAZ1_201707091924_002_0083_L1A.tif");
//
//    std::vector<cv::KeyPoint> kp_all1, kp_all2, kp_block1, kp_block2, kp_orb1, kp_orb2;
//    cv::Mat des_all1, des_all2, des_block1, des_block2, des_orb1, des_orb2;
//    std::vector<cv::KeyPoint> kp_all1_g, kp_all2_g, kp_block1_g, kp_block2_g, kp_orb1_g, kp_orb2_g;
//
//    allMethod(img1, kp_all1, des_all1, "33_kp_all.jpg");
//    blockMethod(img1, kp_block1, des_block1, "33_kp_block.jpg");
//    orbMethod(img1, kp_orb1, des_orb1, "33_kp_orb.jpg");
//
//    allMethod(img2, kp_all2, des_all2, "83_kp_all.jpg");
//    blockMethod(img2, kp_block2, des_block2, "83_kp_block.jpg");
//    orbMethod(img2, kp_orb2, des_orb2, "83_kp_orb.jpg");
//
//    bfMatch(kp_all1, des_all1, kp_all2, des_all2, kp_all1_g, kp_all2_g);
//    bfMatch(kp_block1, des_block1, kp_block2, des_block2, kp_block1_g, kp_block2_g);
//    bfMatch(kp_orb1, des_orb1, kp_orb2, des_orb2, kp_orb1_g, kp_orb2_g);
//
//    Mat img_all;
//    img1.copyTo(img_all);
//    for (int i = 0; i < kp_all1_g.size(); ++i) {
//        circle(img_all, kp_all1_g[i].pt, 2, cv::Scalar(0, 255, 255), 2);
//    }
//    cv::imwrite("33_all_match.jpg", img_all);
//
//    Mat img_block;
//    img1.copyTo(img_block);
//    for (int i = 0; i < kp_block1_g.size(); ++i) {
//        circle(img_block, kp_block1_g[i].pt, 2, cv::Scalar(0, 255, 255), 2);
//    }
//    cv::imwrite("33_block_match.jpg", img_block);
//
//    Mat img_orb;
//    img1.copyTo(img_orb);
//    for (int i = 0; i < kp_orb1_g.size(); ++i) {
//        circle(img_orb, kp_orb1_g[i].pt, 2, cv::Scalar(0, 255, 255), 2);
//    }
//    cv::imwrite("33_orb_match.jpg", img_orb);

//    Mat homo = findHomography(kp_all1_g, kp_all2_g);
//    cout << homo << endl;


}