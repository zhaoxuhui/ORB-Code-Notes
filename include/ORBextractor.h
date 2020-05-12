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

// 防卫式声明
#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

// 两个基本的STL头文件
#include <vector>
#include <list>
// OpenCV的核心函数头文件
#include <opencv/cv.h>


namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    // 构造函数，因为太简单所以就写成这种形式了
    // 只做了一件事情，就是将成员变量bNoMore设为了false
    // 后面也说到了bNoMore用于说明当前节点是否可以继续分割
    ExtractorNode():bNoMore(false){}

    // 拆分当前节点为4个子节点
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    // vector，用于存放当前节点里的特征点
    std::vector<cv::KeyPoint> vKeys;
    // 节点的四个坐标，左上、右上、左下、右下
    cv::Point2i UL, UR, BL, BR;
    // ExtractorNode的迭代器，用于在list中迭代元素
    std::list<ExtractorNode>::iterator lit;
    // bNoMore，用于指明当前节点是否还可以继续分割，默认是false
    // 也就是说默认是可以继续分割的，若为true则不能再分
    bool bNoMore;
};

class ORBextractor
{
public:
    
    // 枚举类型变量
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    // ORBextractor的构造函数
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    // 析构函数
    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    // 重载了括号运算符，方便直接调用
    // 这样新建了一个对象后就可以像调用函数一样调用它了
    // 这也是对外执行ORB特征提取的唯一接口
    // 例如:
    // ORB_SLAM2::ORBextractor orb = ORB_SLAM2::ORBextractor(1000, 1.2, 8, 20, 10);
    // orb(img_gray, cv::Mat(), keypoints, descriptors);
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    // 内联函数，直接返回成员变量，金字塔层数
    int inline GetLevels(){
        return nlevels;}

    // 返回尺度因子
    float inline GetScaleFactor(){
        return scaleFactor;}

    // 返回尺度因子列表
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    // 影像金字塔，里面存放每一层影像的内容
    std::vector<cv::Mat> mvImagePyramid;

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;

    // 特征提取相关参数
    // 它们都是protected的，也就是说外部函数无法直接访问
    // 特征提取数量，这里需要理解的是由于有多层金字塔，
    // 因此这里的特征点数量指的是所有层特征加一起的总数，并不是初始层的特征点个数
    // 初始层的特征点个数可以根据等比数列求和公式算出来，是小于这个数值的
    int nfeatures;
    double scaleFactor; // 金字塔的尺度缩放因子
    int nlevels;    // 金字塔层数
    int iniThFAST;  // 初始FAST阈值
    int minThFAST;  // 最小FAST阈值

    // 用于存储每一层的特征点个数
    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    // 多个vector，用于储存每层的尺度因子和逆尺度相关信息
    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

