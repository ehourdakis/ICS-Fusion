#include "Algorithm/src/Isam.h"
#include <vector>
#include <random>

#include <Eigen/StdVector>
#include<fstream>

#define NN 10
#define NL 1000

#define ADD_NOISE

using namespace Eigen;

const int isam::Pose2d::dim;
std::vector<sMatrix4> poses;
std::vector<sMatrix4> truePoses;

Eigen::Quaterniond euler2quat(float3 &f)
{
    float roll=f.x;
    float pitch=f.y;
    float yaw=f.z;
    Eigen::Quaterniond q;
    q = AngleAxisd(roll, Vector3d::UnitX())
    * AngleAxisd(pitch, Vector3d::UnitY())
    * AngleAxisd(yaw, Vector3d::UnitZ());

    return q;
}

sMatrix4 homo(float3 &trans,float3 R)
{
    Eigen::Quaterniond q=euler2quat(R);
    
    sMatrix4 ret;
    Eigen::Matrix3d rot= q.matrix();
    
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            ret(i,j)=rot(i,j);
    
    ret(0,3)=trans.x;
    ret(1,3)=trans.y;
    ret(2,3)=trans.z;

    return ret;
}

void savePoses(char *fileName,std::vector<sMatrix4> &poses)
{
    using namespace std;
    ofstream file(fileName, std::ios::out);
    Eigen::Matrix3f rot;
    for(uint p=0;p<poses.size();p++)
    {
        sMatrix4 pose=poses[p];
        for (int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                rot(i,j)=pose(i,j);
            }
        }
        Eigen::Vector3f rotV = rot.eulerAngles(0, 1, 2);

        file<<pose(0,3)<<','<<pose(1,3)<<','<<pose(2,3)<<" " ;
        file<<rotV(0)<<','<<rotV(1)<<','<<rotV(2)<<'\n';
    }


    file.close();
}


int main()
{
    Isam *_isam=new Isam();
    unsigned seed=0;
    std::default_random_engine generator (seed);    
    
    float s=0.01;
    float sr=0.01;
    float sl=0.1;
    
    std::normal_distribution<float> landMarkDistr (0.0,sl);
    std::normal_distribution<float> poseDistr (0.0,s);
    std::normal_distribution<float> rotDistr (0.0,sr);
    
    for(int i=0;i<NN;i++)
    {
        float3 trans=make_float3(0.2*i,0,0);
        float3 rot=make_float3(0,0,0);
        
#ifdef ADD_NOISE        
        trans.x+=poseDistr(generator);
        trans.y+=poseDistr(generator);
        trans.z+=poseDistr(generator);
        
        rot.x+=rotDistr(generator);
        rot.y+=rotDistr(generator);
        rot.z+=rotDistr(generator);
#endif        
        sMatrix4 pose=homo(trans,rot);
        
        poses.push_back(pose);
        
        sMatrix4 truePose;
        
        truePose(0,3)=0.2*i;
        
        truePoses.push_back(truePose);
        
        std::cout<<pose<<std::endl;
    }
    
    _isam->init(poses[0]);
    for(int i=1;i<NN;i++)
    {
        //float s=0.0001;
        sMatrix6 cov;
        cov=cov*s;
        cov(3,3)=sr;
        cov(4,4)=sr;
        cov(5,5)=sr;
        sMatrix4 p=poses[i];
        _isam->addFrame(p,cov);
    }    
    
    for(int i=0;i<NL;i++)
    {
        float3 l;
        l.x=0.01*i;
        l.z=0.1;        
        _isam->addLandmark(l);
    }
    
    for(int lid=0;lid<NL;lid++)
    {
        
        for(int pid=0;pid<NN;pid++)
        {
            float3 l;
            l.z=0.1;
            l.x=0.01*lid-0.2*pid;

#ifdef ADD_NOISE                         
            l.x+=landMarkDistr(generator);
            l.y+=landMarkDistr(generator);
            l.z+=landMarkDistr(generator);            
#endif
            
            sMatrix3 cov;
            cov=cov*sl;
            
            _isam->connectLandmark(l,lid,pid,cov);
        }
    }
    
    _isam->optimize(NN);
    
    std::cout<<"=============================="<<std::endl;
    
    for(int i=0;i<_isam->poseSize();i++)
    {
        sMatrix4 isamPose=_isam->getPose(i);
        sMatrix4 truePose=truePoses[i];
        sMatrix4 voPose=poses[i];
        
        float2 isamErr=checkPoseErr(isamPose,truePose);
        float2 voErr=checkPoseErr(voPose,truePose);
        
        std::cout<<isamErr.x<<","<<isamErr.y<<std::endl;
        std::cout<<voErr.x<<","<<voErr.y<<std::endl;
        std::cout<<std::endl;        
        
    }
    
    char buf[32];
    sprintf(buf,"f_%d_poses",NN);
    savePoses(buf,truePoses);
    
    return 0;
}
