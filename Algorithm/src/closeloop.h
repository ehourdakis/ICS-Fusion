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
        bool addFrameWithPose(uint16_t *depth,uchar3 *rgb,sMatrix4 gt);

        IcsFusion* fusion() const
        {
            return _fusion;
        }

        sMatrix4 getPose() const;
        bool optimize();
        float findTransformation(sMatrix4 &tr);
        
        Image<float3, Host> getAllVertex() const;
        
        int getPoseGraphIdx() const;
        void reInit();

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

        void saveIcpCov(char *fileName) const;
        void saveDescrCov(char *fileName) const;

        std::vector<float3> getKeypts() const
        {
            return lastKeyPts;
        }

        keyptsMap *getKeyMap() const
        {
            return _keyMap;
        }

        void clearFirsts(int idx);

    private:
        
        void calcIsamPoses();
        
        IcsFusion *_fusion;
        PoseGraph *_isam;
        int prevKeyPoseIdx;
        int passedFromLastKeyFrame;

        std::vector<float3> lastKeyPts;
        std::vector<FeatDescriptor> lastDescr;

        const kparams_t &params;
        int _frame;
        
        void fixMap();

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
        
        std::vector<sMatrix4> isamVec;

//         SmoothNet *smoothNet;

};

#endif // CLOSELOOP_H


