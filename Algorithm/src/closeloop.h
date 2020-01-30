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
        CloseLoop(kparams_t p, sMatrix4 initPose);
        ~CloseLoop();

        bool preprocess(uint16_t *depth,uchar3 *rgb);
        bool preprocess(float *depth,uchar3 *rgb);

        bool processFrame();
        bool addFrameWithPose(uint16_t *depth,uchar3 *rgb,sMatrix4 gt);

        IcsFusion* fusion() const
        {
            return _fusion;
        }

        sMatrix4 getPose() const;
        bool optimize();
        bool addPoseConstrain(const sMatrix4 &pose);
    private:
        sMatrix4 firstPose;
        sMatrix4 prevPose;
        IcsFusion *_fusion;
        PoseGraph *_isam;

        kparams_t params;
        int _frame;

        //save data for de-integration
        std::vector<DepthHost> depths;
        std::vector<RgbHost> rgbs;
        std::vector<sMatrix4> poses;

        void clear();
        

};

#endif // CLOSELOOP_H


