#ifndef FEATUREDETECTOR_H
#define FEATUREDETECTOR_H

#include"icsFusion.h"
#include"kparams.h"
#include"Isam.h"
#include"featursDescriptor.h"
#include"SiftCovEstimator.h"
//=====OpenCv=====
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>

//typedef cv::Mat descr_t;

#include"sift/sift_cov.h"
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

        void getFeatImage(uchar3 *out, std::vector<cv::DMatch> &good_matches);
        
        void saveImage(char *filename) const;
    private:
        cv::Mat cvOutput;
        uchar3 *drawnDesc;
        uchar3 *oldDrawnDesc;

        //cv::Mat cvRgb,oldCvRgb;
        double focusThr;
        double oldFocusMeasure;

        cv::Ptr<cv::Feature2D> sift;
        cv::Ptr<cv::FlannBasedMatcher> matcher;
        SiftCovEstimator *covEstim;

        //cv keypoint type
        std::vector<cv::KeyPoint> cvKeypoints;


        std::vector<cv::KeyPoint> oldCvKeypoints;
        std::vector<float3> oldKeypts3D;

        Eigen::MatrixXd computeCov2DTo3D(Eigen::MatrixXd cov2D,
                                                          double depth,
                                                          double fx,
                                                          double fy,
                                                          double cx,
                                                          double cy,
                                                          double depth_noise_cov);

        void calcMask(DepthHost &depth,cv::Mat &mask);

        IcsFusion *_fusion;
        PoseGraph *_isam;
        kparams_t _params;

        float ratio_thresh;
};

#endif // FEATUREDETECTOR_H
