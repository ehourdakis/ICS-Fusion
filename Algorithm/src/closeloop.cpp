#include "closeloop.h"
#include<iostream>
#include"utils.h"

//#define SAVE_VOXEL_GRID

//#define USE_G2O
#ifdef USE_G2O
#include"g2oGraph.h"
#endif

#include"constant_parameters.h"
#include <unistd.h>
CloseLoop::CloseLoop(kparams_t p,sMatrix4 initPose)
    :params(p),
     _frame(0)
{
    _fusion = new IcsFusion(params,initPose);

    uint3 voxelSize;
    voxelSize.x=params.volume_direction.x/params.volume_resolution.x;
    voxelSize.y=params.volume_direction.y/params.volume_resolution.y;
    voxelSize.z=params.volume_direction.z/params.volume_resolution.z;

    sliceSize.x=voxelSize.x*params.voxelSliceSize.x;
    sliceSize.y=voxelSize.y*params.voxelSliceSize.y;
    sliceSize.z=voxelSize.z*params.voxelSliceSize.z;

#ifdef USE_G2O
    _isam=new G2oGraph(params);
#else
    _isam=new Isam();
#endif

    _featDet=new FeatureDetector(params,_fusion,_isam);
    _keyMap=new keyptsMap(_isam,_fusion);
    firstPose=initPose;


    minRot(0,0)=0.8536;
    minRot(0,1)=-0.2183;
    minRot(0,2)=0.4731;

    minRot(1,0)=0.3536;
    minRot(1,1)=0.9096;
    minRot(1,2)=-0.2183;

    minRot(2,0)=-0.3827;
    minRot(2,1)=0.3536;
    minRot(2,2)=0.8536;


    maxRot(0,0)=0.0;
    maxRot(0,1)=0.0;
    maxRot(0,2)=-1.0;

    maxRot(1,0)=0.0;
    maxRot(1,1)=1.0;
    maxRot(1,2)=0.0;

    maxRot(2,0)=1.0;
    maxRot(2,1)=0.0;
    maxRot(2,2)=0.0;


    minTrans=0.3;
    maxTrans=0.3;


    smoothNet=new SmoothNet(_fusion);
}

//For testing purposes only.
//Do not run icp just integrate data in pose gt
bool CloseLoop::addFrameWithPose(uint16_t *depth,uchar3 *rgb,sMatrix4 gt)
{
  _fusion->preprocessing(depth,rgb);
  _fusion->setPose(gt);
  _fusion->integration(_frame);
  _fusion->raycasting(_frame);
  _frame++;
  return true;
}

bool CloseLoop::preprocess(uint16_t *depth,uchar3 *rgb)
{
    _fusion->preprocessing(depth,rgb);
    return true;
}

bool CloseLoop::preprocess(float *depth,uchar3 *rgb)
{
    _fusion->preprocessing2(depth,rgb);
    return true;
}


bool CloseLoop::processFrame()
{
    std::cout<<"[FRAME="<<_frame<<"]"<<std::endl;

    bool tracked=_fusion->tracking(_frame);
    bool integrated=_fusion->integration(_frame);

    if(!tracked)
    {
        std::cerr<<"[FRAME="<<_frame<<"] Tracking faild!"<<std::endl;
    }
    if(!integrated)
    {
        std::cerr<<"[FRAME="<<_frame<<"] Integration faild!"<<std::endl;        
    }


    if(_frame==3)
    {
        _isam->init(_fusion->getPose() );
        prevPose=_fusion->getPose();
        isamPoses.push_back(prevPose);
    }
    else if(_frame>3 && tracked)
    {
        sMatrix4 pose=_fusion->getPose();
        DepthHost rawDepth;
        _fusion->getDepthRaw(rawDepth);
        depths.push_back(rawDepth);

        RgbHost rawRgb;
        _fusion->getImageRaw(rawRgb);
        rgbs.push_back(rawRgb);

        poses.push_back(pose);
        //calculate covariance before raycast
        sMatrix6 icpCov =_fusion->calculate_ICP_COV();
        _isam->addFrame(pose,icpCov);
    }

    bool raycast=_fusion->raycasting(_frame);
    if(!raycast)
    {
        std::cerr<<"[FRAME="<<_frame<<"] Raycast faild!"<<std::endl;
    }

    _frame++;

    return true;
}

bool CloseLoop::fixMap()
{
//    _fusion->updateVolume();
}

sMatrix4 CloseLoop::fixPoses(sMatrix4 fixPose)
{

    _fusion->reset();
    //for(int i=0;i<_isam->poseSize();i++)
    //for(int i=0;i<isamPoses.size();i++)
    for(int i=0;i<poses.size();i++)
    {
        //sMatrix4 pose=_isam->getPose(i);
        sMatrix4 pose=poses[i];
        //sMatrix4 pose=isamPoses[i];
        _fusion->reIntegration(pose,depths[i],rgbs[i]);
    }
    sMatrix4 finalPose=poses[poses.size()-1];
    _fusion->setPose(finalPose);
    return finalPose;
}

bool CloseLoop::addPoseToIsam(VolumeSlices &sl)
{
    sMatrix6 cov;
    cov=cov*1e4;

    sMatrix4 p=_fusion->getPose();
    _isam->addFrame(p,cov);
    prevPose=p;

    DepthHost rawDepth;
    _fusion->getDepthRaw(rawDepth);
    depths.push_back(rawDepth);

    RgbHost rawRgb;
    _fusion->getImageRaw(rawRgb);
    rgbs.push_back(rawRgb);

    slices.push_back(sl);
    poses.push_back(_fusion->getPose());

    return true;
}

bool CloseLoop::featuresMatching()
{
    std::vector<float3> keypoints;
    std::vector<FeatDescriptor> descriptors;

    _featDet->detectFeatures(_frame,depths.back(),rgbs.back(),keypoints,descriptors);

    std::cout<<"Keypts size:"<<keypoints.size()<<std::endl;
    if(_keyMap->isEmpty() )
    {
        _keyMap->addKeypoints(keypoints,descriptors);
        std::cout<<"Keypts added"<<std::endl;
        saveDescData(keypoints,descriptors);
        return false;
    }

    _keyMap->matching(keypoints,descriptors,_frame);
    std::cout<<"Keypts matched"<<std::endl;
    saveDescData(keypoints,descriptors);
    return true;
}

bool CloseLoop::optimize()
{
    double err=_isam->optimize(_frame);

    //add poses for logging
    isamPoses.clear();
    for(int i=0;i<_isam->poseSize();i++)
    {
        isamPoses.push_back(_isam->getPose(i));
    }

    //save log data
    char buf[32];
    sprintf(buf,"f_/f_%d_poses",_frame);
    savePoses(buf,poses);

    sprintf(buf,"f_/f_%d_poses2",_frame);
    savePoses(buf,isamPoses);

}

sMatrix4 CloseLoop::doLoopClosure(sMatrix4 gt)
{
    std::cout<<"Loop closure "<<_frame<<std::endl;
    std::cout<<std::endl;

    bool flag=true;

    if(_frame==4)
    {
        _isam->init(_fusion->getPose() );
        prevPose=_fusion->getPose();
        isamPoses.push_back(prevPose);
    }
    else
    {
        sMatrix6 cov;
        /*
        cov=cov*1e+2;
        //cov=icpCov;
        float f;
        if(tracked)
            f=_fusion->getWrongNormalsSize();
        else
            f=10;
        cov=cov*powf(2,f);
        */
        cov=cov*1e4;
        
        sMatrix4 p=_fusion->getPose();
        _isam->addFrame(p,cov);
        prevPose=p;
    }
    

    DepthHost rawDepth;
    _fusion->getDepthRaw(rawDepth);
    depths.push_back(rawDepth);

    RgbHost rawRgb;
    _fusion->getImageRaw(rawRgb);
    rgbs.push_back(rawRgb);

    poses.push_back(_fusion->getPose());

    std::vector<float3> keypoints;
    std::vector<FeatDescriptor> descriptors;

    _featDet->detectFeatures(_frame,rawDepth,rawRgb,keypoints,descriptors);

    std::cout<<"Keypts size:"<<keypoints.size()<<std::endl;
    if(_keyMap->isEmpty() )
    {
        _keyMap->addKeypoints(keypoints,descriptors);
        std::cout<<"Keypts added"<<std::endl;
        return gt;

    }
    else
    {
        _keyMap->matching(keypoints,descriptors,_frame);
        std::cout<<"Keypts matched"<<std::endl;
        double err=_isam->optimize(_frame);
        isamPoses.clear();
        isamPoses.push_back(_isam->getPose(1));

        for(int i=0;i<_isam->poseSize();i++)
        {
            isamPoses.push_back(_isam->getPose(i));
        }
        //fixPoses(gt);
    }

    char buf[32];
    sprintf(buf,"f_/f_%d_poses",_frame);
    savePoses(buf,poses);

    sprintf(buf,"f_/f_%d_poses2",_frame);
    savePoses(buf,isamPoses);

    saveDescData(keypoints,descriptors);

    /*
    clear();
    _isam->init(_fusion->getPose());
    _keyMap->addKeypoints(keypoints,descriptors);
    poses.push_back(_fusion->getPose());
    */
    return _fusion->getPose();
}


sMatrix4 CloseLoop::getPose() const
{
//    if(_frame<5)
        return _fusion->getPose();

    int poseIdx=_isam->poseSize()-1;
    return _isam->getPose(poseIdx);
}

void CloseLoop::saveDescData(const std::vector<float3> &keypts, const std::vector<FeatDescriptor> &desc)
{
    char buf[32];

    sprintf(buf,"f_/f_%d_pose",_frame);
    sMatrix4 pose=_fusion->getPose();
    savePose(buf,pose);

#ifdef SAVE_VOXEL_GRID
    sprintf(buf,"f_/f_%d_voxels",_frame);
    Volume v=_fusion->getVolume();
    saveVoxelsToFile(v,params,std::string(buf) );
#endif
}

void CloseLoop::savePose(char *fileName,const sMatrix4 &pose)
{
    using namespace std;
    ofstream file(fileName, std::ios::out);
    Eigen::Matrix3f rot;
    for (int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            file<<pose(i,j)<<" ";
        }
        file<<'\n';
    }
    file.close();
}

void CloseLoop::savePoses(char *fileName,std::vector<sMatrix4> &poses,sMatrix4 fp)
{
    using namespace std;
    ofstream file(fileName, std::ios::out);
    
    Eigen::Matrix3f rot;
    for (int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            rot(i,j)=firstPose(i,j);
        }
    }
    Eigen::Vector3f rotV = rot.eulerAngles(0, 1, 2);

    file<<firstPose(0,3)<<','<<firstPose(1,3)<<','<<firstPose(2,3)<<" " ;
    file<<rotV(2)<<','<<rotV(1)<<','<<rotV(0)<<'\n';

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

    //add fix pose
    sMatrix4 pose=fp;
    for (int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            rot(i,j)=pose(i,j);
        }
    }
    rotV = rot.eulerAngles(0, 1, 2);

    file<<pose(0,3)<<','<<pose(1,3)<<','<<pose(2,3)<<" " ;
    file<<rotV(0)<<','<<rotV(1)<<','<<rotV(2)<<'\n';
        
    
    
    file.close();
}

void CloseLoop::savePoses(char *fileName,std::vector<sMatrix4> &poses)
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

void CloseLoop::clear()
{    
    /*
    for(uint i=0; i<poses.size();i++ )
    {
        Host depth=depths[i];
        Host rgb=rgbs[i];

        depth.release();
        rgb.release();

    }
    */
    _keyMap->clear();

//    depths.clear();
//    rgbs.clear();
    //poses.clear();
    //isamPoses.clear();
    _isam->clear();
}


bool CloseLoop::isKeyFrame() const
{    
//    return false;
    if(_frame==4)
        return true;

    return _frame > 4 && (_frame % 30 == 0);

    float minTr=0.3;
    float maxTr=0.5;

    float minR=0.7854;
    float maxR=1.5708;

    sMatrix4 pose=_fusion->getPose();
    float2 err=checkDeltaPoseErr(prevPose,pose);

    if(err.x<minTr && err.y<minR)
        return false;

    if(err.y>maxTr && err.y>maxR)
        return true;

    return false;

    //return checkKeyFrameDeltaPose();


    
//    if(_frame==180)
//        return true;
    return false;
    //return _frame > 4 && (_frame % 10 == 0);
}

float2 CloseLoop::checkDeltaPoseErr(sMatrix4 p1,sMatrix4 p2)
{
    float2 ret;
    sMatrix3 r1,r2;

    float3 tr1=make_float3(p1(0,3),p1(1,3),p1(2,3));
    float3 tr2=make_float3(p2(0,3),p2(1,3),p2(2,3));

    tr1=tr1-tr2;
    ret.x=l2(tr1);

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            r1(i,j)=p1(i,j);
            r2(i,j)=p2(i,j);
        }
    }
    r1=r1*transpose(r2);
    float3 f=logMap(r1);

    ret.y=l2(f);
    return ret;
}

bool CloseLoop::checkKeyFrameDeltaPose() const
{
    sMatrix4 p=inverse(prevPose)*_fusion->getPose();
    float trL2=sqrt( sq(p(0,3))+sq(p(1,3))+sq(p(2,3)) );
    
    
    
    float rot[3];
    eulerFromHomo(p,rot[0],rot[1],rot[2]);
    float rotInfNorm=rot[0];
    
    for(int i=0;i<3;i++)
    {
        if(rot[i]>rotInfNorm)
            rotInfNorm=rot[i];
    }
    
    if(trL2>0.5)
        return true;
    
    
    // PI/4
    
    float piDiv4=PI/6;
    /*
    float a = rotInfNorm - piDiv4
    a += (a>PI) ? -PI : (a<-PI) ? 2*PI : 0*/

    std::cout<<"TR:"<<trL2<<" R:"<<rot[0]<<","<<rot[1]<<","<<rot[2]<<" "<<rotInfNorm<<std::endl;
    if(rotInfNorm > piDiv4)
        return true;
    return false;
    
}

CloseLoop::~CloseLoop()
{
    clear();
    delete _featDet;
    delete _fusion;
    delete _isam;
}

void CloseLoop::saveKeypoints(std::string fileName,const std::vector<float3> &keypts)
{
    std::ofstream keypts_out_file(fileName, std::ios::out);

    for (int i = 0; i < keypts.size(); i++)
    {
        keypts_out_file<<keypts[i].x<<" "
                       <<keypts[i].y<<" "
                       <<keypts[i].z<<"\n";
    }
    keypts_out_file.close();
}

void CloseLoop::saveDescriptors(std::string fileName, const std::vector<FeatDescriptor> &desc)
{
    std::ofstream desc_out_file(fileName, std::ios::out);

//    desc_out_file<<(float)desc.size()<<'\n';
//    desc_out_file<<(float)FeatDescriptor::size()<<'\n';
    for(int i=0;i<desc.size();i++)
    {
        for(uint j=0;j<FeatDescriptor::size();j++)
        {
            desc_out_file<<desc[i].data[j]<<" ";
        }
        std::cout<<'\n';

    }
    desc_out_file.close();
}
