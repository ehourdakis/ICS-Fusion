#ifndef ISAM_H
#define ISAM_H

// #include<mrpt/poses/CPose3DPDFGaussian.h>
// #include <mrpt/poses/CPose3D.h>

#include"utils.h"
#include<isam/slam3d.h>
#include <isam/Properties.h>
#include <isam/isam.h>
#include<isam/Point3d.h>
#include"PoseGraph.h"
#include<isam/Pose3d.h>
class Isam :public PoseGraph
{
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        //Isam(const sMatrix4 &initalPose);
        Isam();
        void addFrame(const sMatrix4 &pose,const sMatrix6 &cov);
        //void addLandmark(float3 pos1,float3 pos2,double dist);


        //Adds a landmark and return its idx
        virtual int addLandmark(float3 pos) override;
        virtual void connectLandmark(float3 pos,int landIdx,int poseIdx, sMatrix3 &cov) override;

        virtual void init(const sMatrix4 &initalPose) override;
        virtual void clear() override;

        void addFixPose(const sMatrix4 &fixPose);
        double optimize(int frame) override;
        void addPoseConstrain(int p1,int p2,const sMatrix4 &delta, const sMatrix6 &cov);
        //void convertCovariance(sMatrix4 pose,sMatrix6 cov) override;


        virtual sMatrix4 getPose(int i)  override;

        virtual uint poseSize() const override
        {
            return pose_nodes.size();
        }

        virtual uint landmarksSize() const override
        {
            return landmarks.size();
        }

        static isam::Pose3d toIsamPose(const sMatrix4 &pose);
        static sMatrix4 fromIsamNode(isam::Pose3d_Node *node);


        static isam::Point3d toIsamPoint(const float3 &f);

    private:

//        void addLandmarks(std::vector<float3> &keypoints,descr_t &descriptors);

        isam::Slam *slam;
        std::vector<isam::Pose3d_Node*> pose_nodes;
        std::vector<isam::Factor*> factors;
        std::vector<isam::Point3d_Node*> landmarks;

        static Eigen::MatrixXd toEigen(sMatrix4 mat);
        static Eigen::MatrixXd toEigen(sMatrix6 mat);
        static Eigen::MatrixXd toEigen(sMatrix3 mat);
        sMatrix4 prevPose;

        /*
        void transformToFrame(sMatrix4 &delta,sMatrix6 &cov);
        mrpt::poses::CPose3DPDFGaussian toCPose3DPDF(sMatrix4 pose,sMatrix6 cov);
        mrpt::poses::CPose3D toCPose(sMatrix4 pose);
        */
};
#endif
