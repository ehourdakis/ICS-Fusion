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

class CloseLoop
{
    public:
        CloseLoop(kparams_t p, sMatrix4 initPose);
        ~CloseLoop();

        bool addFrame(uint16_t *depth,uchar3 *rgb);
        bool addFrameWithPose(uint16_t *depth,uchar3 *rgb,sMatrix4 gt);

        bool isKeyFrame() const;

        IcsFusion* fusion() const
        {
            return _fusion;
        }
        sMatrix4 doLoopClosure(sMatrix4 pose);
        sMatrix4 fixPoses(sMatrix4 fixPose);
        void findFrameDescr();

        FeatureDetector* getFeatDetector()
        {
            return _featDet;
        }

        sMatrix4 getPose() const;

        bool checkKeyFrameDeltaPose() const;

        static float2 checkDeltaPoseErr(sMatrix4 p1,sMatrix4 p2);
    private:
        sMatrix4 firstPose;
        sMatrix4 prevPose;
        IcsFusion *_fusion;
        PoseGraph *_isam;
        FeatureDetector *_featDet;
        keyptsMap *_keyMap;

        kparams_t params;
        int _frame;

        //save data for de-integration
        std::vector<DepthHost> depths;
        std::vector<RgbHost> rgbs;
        std::vector<sMatrix4> poses;
        std::vector<sMatrix4> isamPoses;
        std::vector<sMatrix4> isamPoses2;
        sMatrix6 icpCov;

        void clear();
        void saveDescData(const std::vector<float3> &keypts,const std::vector<FeatDescriptor> &desc);
        void savePoses(char *fileName, std::vector<sMatrix4> &poses, sMatrix4 fp);
        void savePoses(char *fileName,std::vector<sMatrix4> &poses);
        void saveDescriptors(std::string fileName, const std::vector<FeatDescriptor> &desc);
        void saveKeypoints(std::string fileName,const std::vector<float3> &keypts);
        void savePose(char *fileName,const sMatrix4 &pose);

        bool tracked;

        sMatrix3 minRot;
        sMatrix3 maxRot;

        float minTrans;
        float maxTrans;

        uint3 sliceSize;



        //check if volume need sift
        bool needSift() const;

};

#endif // CLOSELOOP_H


