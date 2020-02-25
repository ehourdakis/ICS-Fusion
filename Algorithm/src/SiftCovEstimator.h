#ifndef SIFT_COV_ESTIMATOR_H 
#define SIFT_COV_ESTIMATOR_H

#include"sift/sift.h"
#include"sift/sift_cov.h"
#include<sMatrix.h>
#include <Eigen/Dense>

class SiftCovEstimator
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SiftCovEstimator(int width,int height,float thr,bool draw);
        ~SiftCovEstimator()
        {
            clear();
        }

        void load(void *data,void *rgb);
        bool calcCovariance(const cv::KeyPoint &pt, Eigen::Matrix2d &cov);
        void getDrawnData(void *out);
        void clear();
    private:

        float cv2SMatrix(CvMat *m, Eigen::Matrix2d &cov2D);

        int octaves=0, intervals=0;

        bool _draw;
        IplImage *_frame;
        IplImage *_drawFrame;
        CovEstimator *_estimator;

         IplImage*** imgPyr;

        int _width;
        int _height;
        float _thr;
};
#endif
