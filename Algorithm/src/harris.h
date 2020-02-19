#ifndef HARRIS_H
#define HARRIS_H


#include"utils.h"

#include"image.h"
//=====OpenCv=====
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>

class Harris
{
    public:
        Harris();


        void detectCorners(VertHost &vert,
                           RgbHost &rgb,
                           std::vector<int> &points,
                           std::vector<float3> &keypts3D);

        void showKeypts(cv::Mat &outMat);

    private:
        //TODO add this to params
        int blockSize;
        int apertureSize;
        float k;
        float thresh;

        cv::Mat dst_norm;
        cv::Mat dst_norm_scaled;
        cv::Mat cvRgb;
        cv::Mat cvGrey ;
        cv::Mat dst;

        std::vector< cv::Point2f > corners;
};

#endif
