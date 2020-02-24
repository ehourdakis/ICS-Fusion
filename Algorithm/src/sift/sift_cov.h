#ifndef SIFT_COV_H
#define SIFT_COV_H

#include <iostream>
#include <string>

//openCV
#include <cv.h>
#include <cxcore.h>

//own
#include "ImagePyramid.h"
#include "CovEstimator.h"
#include "CovEstArgumentEvaluator.h"
#include "GeneralFeature.h"

// covUtils
#include "ipoint.h"
#include "covUtils.h"

// from SIFT detector

#include "sift.h"
#include "imgfeatures.h"
//#include<utils.h>
#include <opencv2/core/types_c.h>

void cvSeq2Ipts(const CvSeq* keypoints, const CvSeq* descriptors, IpVec& ipts);
void feature2Ipts(struct feature* features, int nrFeat, IpVec& ipts);

class SiftCov 
{
    public:
        SiftCov();
        SiftCov(const char *sfilename, bool draw = false);
        void load(int width, int height, void *data, void *rgb);
        unsigned int detectFeatures() ;
        void getCovariance();
        void draw(int width, int height, void *data);

        const IpVec& getFeatures() const
        {
            return ipts;
        }

         IplImage *getFrame()
         {
             return frame;
         }


    private:
        IpVec ipts;
        string filename;
        IplImage *frame;
        IplImage *drawFrame;
        IplImage*** imgPyr;

        int octaves;
        int intervals;
        CovEstimator *estimator;
};
#endif
