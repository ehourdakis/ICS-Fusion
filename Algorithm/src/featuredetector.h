#ifndef FEATUREDETECTOR_H
#define FEATUREDETECTOR_H

#include"icsFusion.h"
#include"kparams.h"
#include"Isam.h"
#include"featursDescriptor.h"

//=====OpenCv=====
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>

//typedef cv::Mat descr_t;
class FeatureDetector
{
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        FeatureDetector(kparams_t p, IcsFusion *f, PoseGraph *isam);

        void detectFeatures(int frame,
                            DepthHost &depth,
                            RgbHost &rgb,
                            std::vector<float3> &keypoints,
                            std::vector<FeatDescriptor> &descr);
        void getFeatImage(uchar3 *out);

        void getFeatImage(cv::Mat &outMat, std::vector<cv::DMatch> &good_matches);
    private:
        void getDescrFromMat(int row,cv::Mat &mat,FeatDescriptor &descr);
        void calcMask(DepthHost &depth,cv::Mat &mask);

        bool drawNewData;
        IcsFusion *_fusion;
        PoseGraph *_isam;
        kparams_t _params;

        void writePointsPcd(Image<float3, Host> &vertex,
                      Image<float3, Host> &norm,
                      char *filename);
        void writeNormalsPcd(Image<float3, Host> &vertex,
                      Image<float3, Host> &norm,
                      char *filename);

        void writeWrongNormPoints(char *fileName);
        void writeWrongNormNormals(char *fileName);
        void writePoints(std::vector<float3> &vec,char *fileName);

        //opencv
        cv::Mat cvRgb,cvGrey,cvOutput;
        cv::Ptr<cv::Feature2D> sift;
        cv::Ptr<cv::FlannBasedMatcher> matcher;


        //cv keypoint type
        std::vector<cv::KeyPoint> cvKeypoints;

        //        std::vector<cv::KeyPoint> cvKeypoints;
        std::vector<cv::KeyPoint> oldCvKeypoints;
        std::vector<float3> oldKeypts3D;
//        cv::Mat oldDescriptors;
        cv::Mat oldCvRgb;

        float ratio_thresh;

        int _frame;
};

#endif // FEATUREDETECTOR_H
