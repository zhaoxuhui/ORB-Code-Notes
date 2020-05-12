//
// Created by root on 20-2-14.
//


#include <opencv/cv.hpp>
#include "MIXextractor.h"

using namespace cv;
using namespace std;

namespace ORB_SLAM2 {

    MIXextractor::MIXextractor(int nfeatures) : nfeatures(nfeatures) {

    }

    void MIXextractor::operator()(const cv::InputArray &image, std::vector<cv::KeyPoint> &keypoints,
                                  const cv::OutputArray &descriptors) {
        if (image.empty())
            return;

        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create(nfeatures);
        cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> bf = cv::xfeatures2d::BriefDescriptorExtractor::create();

        if (image.type() != CV_8UC1) {
            Mat image_gray;
            cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
            sift->detect(image_gray, keypoints);
            bf->compute(image_gray, keypoints, descriptors);
        } else {
            sift->detect(image, keypoints);
            bf->compute(image, keypoints, descriptors);
        }
    }


    void MIXextractor::operator()(cv::Mat image, std::vector<cv::KeyPoint> &keypoints, cv::Mat descriptors, int rows,
                                  int cols) {
        vector<vector<int>> blocks;

        int width = image.size().width;
        int height = image.size().height;
        int block_width = width / cols;
        int block_height = height / rows;
        for (int i = 0; i < cols; ++i) {
            for (int j = 0; j < rows; ++j) {
                vector<int> index;
                index.push_back(block_width * i);
                index.push_back(block_width * (i + 1));
                index.push_back(block_height * j);
                index.push_back(block_height * (j + 1));
                blocks.push_back(index);
            }
        }

        int nKpsPerBlock = nfeatures / (cols * rows);

        for (int i = 0; i < blocks.size(); ++i) {
            vector<cv::KeyPoint> tmpKp;
            cv::Mat tmpDes;
            vector<int> index = blocks[i];
            cv::Mat block = image(cv::Range(index[2], index[3]), cv::Range(index[0], index[1]));
            int num = 0;
            int iter_num = nKpsPerBlock;
            int counter = 0;
            while (num < nKpsPerBlock) {
                getMixKps(block, iter_num, tmpKp, tmpDes);
                num = tmpKp.size();
                iter_num += int(0.3 * nKpsPerBlock);
                counter += 1;
                if (counter > 10) {
                    break;
                }
            }

            for (int j = 0; j < tmpKp.size(); ++j) {
                tmpKp[j].pt.x += index[0];
                tmpKp[j].pt.y += index[2];
            }
            keypoints.insert(keypoints.end(), tmpKp.begin(), tmpKp.end());
            if (i == 0) {
                descriptors = tmpDes;
            } else {
                vconcat(descriptors, tmpDes, descriptors);
            }
        }

    }

    void MIXextractor::getMixKps(const cv::InputArray &image, int numKps, std::vector<cv::KeyPoint> &keypoints,
                                 const cv::OutputArray &descriptors) {
        if (image.empty())
            return;

        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create(numKps);
        cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> bf = cv::xfeatures2d::BriefDescriptorExtractor::create();

        if (image.type() != CV_8UC1) {
            Mat image_gray;
            cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
            sift->detect(image_gray, keypoints);
            bf->compute(image_gray, keypoints, descriptors);
        } else {
            sift->detect(image, keypoints);
            bf->compute(image, keypoints, descriptors);
        }
    }

}