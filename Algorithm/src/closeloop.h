#ifndef CLOSELOOP_H
#define CLOSELOOP_H

#include"icsFusion.h"
#include"kparams.h"
#include"Isam.h"

#include<vector>
#include"utils.h"
#include"featuredetector.h"
#include"keyptsmap.h"

#include"PoseGraph.h"
#include"smoothnet.h"
#include<list>


#include"harris.h"
#include"Isam.h"

#include"featuredetector.h"

class CloseLoop
{
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CloseLoop(const kparams_t &p, sMatrix4 initPose);
        ~CloseLoop();

        bool preprocess(uint16_t *depth,uchar3 *rgb);
        bool preprocess(float *depth,uchar3 *rgb);

        bool processFrame();
//         bool processKeyFrame();
        bool addFrameWithPose(uint16_t *depth,uchar3 *rgb,sMatrix4 gt);

        IcsFusion* fusion() const
        {
            return _fusion;
        }

        sMatrix4 getPose() const;
        bool optimize();
        float findTransformation(sMatrix4 &tr);
        
        bool findKeyPts(std::vector<int> &evaluation_points, Image<float3, Host> vertices, std::vector<float3> &keyVert);
        Image<float3, Host> getAllVertex() const;
        
        int getPoseGraphIdx() const;
        void reInit();
        bool addTf(int idx,
                   int prevIdx,
                   const sMatrix4 &tf,
                   float fitness,
                   float rmse,
                   const std::vector<int> &source_corr,
                   const std::vector<int> &target_corr,
                   const std::vector<float3> &keyVert,
                   const std::vector<float3> &prevKeyVert);

        void getIsamPoses(std::vector<sMatrix4> &vec);
        void showKeypts(uchar3 *out);
        bool processKeyFrame();
        void getMatches(std::vector<float3> &prevPts,
                        std::vector<float3> &newPts);
        void saveImage(char *filename);
        void saveDescriptors(char *filename);
        void saveKeyPts(char *filename);
        void saveCorrespondance(char *fileName);
        void saveKeyMap(char *descrFile,char *poitsFile,char *frameFile);
    private:
        sMatrix4 firstPose;
        sMatrix4 prevPose;
        IcsFusion *_fusion;
        PoseGraph *_isam;
        int prevKeyPoseIdx;

        std::vector<float3> lastKeyPts;
        std::vector<FeatDescriptor> lastDescr;

        const kparams_t &params;
        int _frame;
        
        void fixMap();
        void reInit(int idx);

        void removeOldNodes(int idx);

        //save data for de-integration
        std::list<DepthHost> depths;
        std::list<RgbHost> rgbs;
        std::list<sMatrix4> poses;
        std::list<sMatrix6> covars;

        bool firstKeyFrame;
        
        bool tracked;

        void clear();


        Harris *harris;
        keyptsMap *_keyMap;
        FeatureDetector *_featDet;

//         SmoothNet *smoothNet;

};

#endif // CLOSELOOP_H


