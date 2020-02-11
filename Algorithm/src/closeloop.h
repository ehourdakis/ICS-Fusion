#ifndef CLOSELOOP_H
#define CLOSELOOP_H

// #include<mrpt/poses/CPose3DPDFGaussian.h>
// #include <mrpt/poses/CPose3D.h>

#include"icsFusion.h"
#include"kparams.h"
#include"Isam.h"

#include<vector>
#include"utils.h"
#include"featuredetector.h"
#include"keyptsmap.h"

#include"PoseGraph.h"
#include"smoothnet.h"
class CloseLoop
{
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CloseLoop(kparams_t p, sMatrix4 initPose);
        ~CloseLoop();

        bool preprocess(uint16_t *depth,uchar3 *rgb);
        bool preprocess(float *depth,uchar3 *rgb);

        bool processFrame();
        bool processKeyFrame();
        bool addFrameWithPose(uint16_t *depth,uchar3 *rgb,sMatrix4 gt);

        IcsFusion* fusion() const
        {
            return _fusion;
        }

        sMatrix4 getPose() const;
        bool optimize();
        float findTransformation(sMatrix4 &tr);
    private:
        sMatrix4 firstPose;
        sMatrix4 prevPose;
        IcsFusion *_fusion;
        PoseGraph *_isam;
        int prevKeyPoseIdx;

        kparams_t params;
        int _frame;
        
        void fixMap();

        //save data for de-integration
        std::vector<DepthHost> depths;
        std::vector<RgbHost> rgbs;
        std::vector<sMatrix4> poses;

        bool firstKeyFrame;

        void clear();
        void reInit();

        SmoothNet *smoothNet;

};

#endif // CLOSELOOP_H


