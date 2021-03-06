#ifndef ICS_FUSION_H
#define ICS_FUSION_H

#include"kparams.h"
#include"utils.h"
#include"volume.h"
#include<vector>

class IcsFusion
{
    public:

        IcsFusion(kparams_t par, float3 initPose)
            :params(par)
        {
            pose = toMatrix4(TooN::SE3<float>(
                       TooN::makeVector(initPose.x, initPose.y, initPose.z, 0, 0, 0)) );
            oldPose=pose;
            this->iterations.clear();
            for (std::vector<int>::iterator it = params.pyramid.begin();it != params.pyramid.end(); it++)
            {
                this->iterations.push_back(*it);
            }
            largestep=0.75*params.mu;
            inverseCam=getInverseCameraMatrix(params.camera);
            camMatrix=getCameraMatrix(params.camera);
            step = min(params.volume_size) / max(params.volume_resolution);
            viewPose = &pose;
            this->languageSpecificConstructor();
        }        

        //Allow a kfusion object to be created with a pose which include orientation as well as position
        IcsFusion(kparams_t par,Matrix4 initPose);

        void restorePose()
        {
            pose=oldPose;
        }

        void languageSpecificConstructor();
        ~IcsFusion();

        void reset();

        bool preprocessing(const ushort * inputDepth,const uchar3 *rgb);
        bool preprocessing2(const float *inputDepth,const uchar3 *rgb) ;

        bool tracking(uint frame);
        bool raycasting(uint frame);
        bool integration(uint frame);

        void getImageProjection(sMatrix4 pose, uchar3 *out);
        float compareRgb();
        float compareRgbTruth(sMatrix4 pose,uchar3 *out );
        void dumpVolume(const  char * filename);
        void renderVolume(uchar3 * out);
        void renderImage(uchar3 * out);

        void renderTrack(uchar3 * out);
        void renderDepth(uchar3 * out);

        void getVertices(std::vector<float3> &vertices);

        sMatrix4 getPose() const
        {
            return pose;
        }

        void setPose(const sMatrix4 pose_)
        {
            pose=pose_;
            forcePose=true;
        }
        void setViewPose(sMatrix4 *value = NULL)
        {
            if (value == NULL)
                viewPose = &pose;
            else
                viewPose = value;
        }
        sMatrix4 *getViewPose()
        {
            return (viewPose);
        }


        float calcNewMapInfo(uchar3 *out);

        Volume getVolume()
        {
            return volume;
        }

        Volume getNewDataVolume()
        {
            return newDataVol;
        }

        void integrateNewData(sMatrix4 p);
        bool deIntegration(sMatrix4 p,const Host &depth,const Host &rgb);
        bool reIntegration(sMatrix4 pose,const Host &depth,const Host &rgb);
        void getImageRaw(Host &to) const;
        void getDepthRaw(Host &data) const;
        
        
        void getIcpValues(Image<float3, Host> &depthVertex,
                          Image<float3, Host> &raycastVertex,
                          Image<float3, Host> &normals,
                          Image<TrackData, Host> &trackData) const;

        sMatrix6 calculate_ICP_COV();
        
        Image<float3, Host> getAllVertex();
        Image<float3, Host> getAllNormals();

        Image<TrackData, Host> getTrackData();
        float getWrongNormalsSize();
        
        Image<float, Host> vertex2Depth();
    private:
        bool _tracked;
        bool forcePose;
        float step;
        sMatrix4 pose;
        sMatrix4 oldPose;
        sMatrix4 deltaPose;
        sMatrix4 trackPose;
        sMatrix4 oldRaycastPose;
        sMatrix4 *viewPose;
        sMatrix4 inverseCam;
        sMatrix4 camMatrix;
        std::vector<int> iterations;
        Volume volume;
        float largestep;
        Volume newDataVol;

        kparams_t params;

        sMatrix4 raycastPose;

        Image<TrackData, Device> reduction;
        Image<float3, Device> vertex, normal;
        std::vector<Image<float3, Device> > inputVertex, inputNormal;
        std::vector<Image<float, Device> > scaledDepth;
        
        Image<sMatrix6, Device> covData;

        Image<float, Device> rawDepth;
        Image<ushort, Device> depthImage;
        Image<uchar3, Device> rawRgb;
        Image<float, HostDevice> output;
        Image<float, Device> gaussian;

        Image<uchar3, Device> renderModel;
//         Image<uchar3, HostDevice>  trackModel, depthModel;




//        Match3D match3d;

        //Functions
        bool updatePoseKernel(sMatrix4 & pose, const float * output,float icp_threshold,sMatrix4 &deltaPose);
        bool checkPoseKernel(sMatrix4 & pose,
                             sMatrix4 oldPose,
                             const float * output,
                             uint2 imageSize,
                             float track_threshold);
};
#endif
