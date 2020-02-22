#ifndef POSE_GRAPH_H 
#define POSE_GRAPH_H

#include"utils.h"
class PoseGraph
{
    public:
        PoseGraph(){}
        virtual ~PoseGraph() {}
        virtual void init(const sMatrix4 &initalPose, const sMatrix6 &cov) = 0;
        virtual void addFrame(const sMatrix4 &pose,const sMatrix6 &cov) = 0;
        virtual int addLandmark(float3 pos) = 0;
        virtual void connectLandmark(float3 pos,int landIdx,int poseIdx, sMatrix3 &cov) = 0;
        virtual void connectLandmark(float2 pos, int lid, double Sigma2){}
        
        virtual void clearLandmarks()
        {

        }

        virtual void clear() = 0;
        virtual void popFront()=0;
        virtual double optimize(int frame) = 0;
        
        virtual sMatrix4 getPose(int i)  = 0;
        virtual uint poseSize() const = 0;
        virtual uint landmarksSize() const = 0;
        
        virtual void addFixPose(const sMatrix4 &fixPose) = 0;
        virtual void addPoseConstrain(int p1,int p2,const sMatrix4 &pose, const sMatrix6 &cov) = 0;
        
        inline bool hasLandmarks() const
        {
            return landmarksSize()>0;
        }
        
};
#endif
