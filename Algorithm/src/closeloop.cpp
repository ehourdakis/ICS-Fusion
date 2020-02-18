#include "closeloop.h"
#include<iostream>
#include"utils.h"

#include"defs.h"

//#define USE_G2O


#ifdef USE_G2O
#include"g2oGraph.h"
#endif

#include"constant_parameters.h"
#include <unistd.h>
#include"kernelscalls.h"

CloseLoop::CloseLoop(const kparams_t &p,sMatrix4 initPose)
    :params(p),
     _frame(-1),
     firstKeyFrame(true)
{
    _fusion = new IcsFusion(params,initPose);

#ifdef USE_G2O
     _isam=new G2oGraph(params);
#else
    _isam=new Isam(params);
#endif
    firstPose=initPose;
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

int CloseLoop::getPoseGraphIdx() const
{
    return _isam->poseSize()-1;
}


bool CloseLoop::addTf(int idx,
                      int prevIdx,
                      const sMatrix4 &tf, 
                      float fitness, 
                      float rmse,
                      const std::vector<int> &source_corr, 
                      const std::vector<int> &target_corr,
                      float3 *keyVert,
                      float3 *prevKeyVert,
                      int size)
{

//     if(fitness<0.1)
//         return false;

    sMatrix6 cov=calculatePoint2PointCov(keyVert,
                                         size,
                                         prevKeyVert,
                                         size,
                                         source_corr,
                                         target_corr,
                                         tf,
                                         params);
    
    //cov=cov*(1/fitness);
    std::cout<<"FITNESS:"<<fitness<<std::endl;
    std::cout<<"RMSE:"<<rmse<<std::endl;
    std::cout<<"COV:\n"<<cov<<std::endl;
    _isam->addPoseConstrain(0,idx,tf,cov);
     optimize();
     removeOldNodes(idx);
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
    _frame++;
//    std::cout<<"[FRAME="<<_frame<<"]"<<std::endl;

    tracked=_fusion->tracking(_frame);
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
        sMatrix4 pose=_fusion->getPose();
        sMatrix6 cov;
        cov=cov*params.cov_small;
        _isam->init(pose,cov);
        prevPose=_fusion->getPose();
         
        DepthHost rawDepth;
        _fusion->getDepthRaw(rawDepth);
        depths.push_back(rawDepth);

        RgbHost rawRgb;
        _fusion->getImageRaw(rawRgb);
        rgbs.push_back(rawRgb);

        covars.push_back(cov);
        poses.push_back(pose);
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
        //float icpFitness=_fusion->getFitness();
        //std::cout<<"ICP Fitness:"<<icpFitness<<std::endl;
        //std::cout<<"ICP Fitness:"<<icpCov<<std::endl;
        //icpCov=icpCov*1000*(1/icpFitness);
        covars.push_back(icpCov);
        _isam->addFrame(pose,icpCov);
    }

    bool raycast=_fusion->raycasting(_frame);
    if(!raycast)
    {
        std::cerr<<"[FRAME="<<_frame<<"] Raycast faild!"<<std::endl;
    }

    return tracked;
}

bool CloseLoop::findKeyPts(std::vector<int> &evaluation_points,int size,Image<float3, Host> vertices,float3 *keyVert)
{
    evaluation_points.clear();
    
    if(!tracked)
        return false;
    
    Image<TrackData, Host> trackData=_fusion->getTrackData();

    std::vector<int> tmp_points;
    uint idx=0;
    uint2 pix;
    for(pix.x=0;pix.x<trackData.size.x;pix.x++)
    {
        for(pix.y=0;pix.y<trackData.size.y;pix.y++)
        {
            if(trackData[pix].result==-5)
            {
                tmp_points.push_back(idx);
            }
            idx++;
        }
        
    }
    if(idx<size)
    {
        return false;
    }

    evaluation_points.reserve(size);
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<int> distr(0, tmp_points.size());

    for(int i=0;i<size;i++)
    {
        int idx=distr(gen);
        while(tmp_points[idx]==-1)
        {
            idx=distr(gen);
        }
        evaluation_points.push_back(tmp_points[idx]);

        uint2 pix;
        pix.x=tmp_points[idx]/vertices.size.y;
        pix.y=tmp_points[idx]%vertices.size.y;
        keyVert[i]=vertices[pix];

        tmp_points[idx]=-1;
    }
    return true;
}

Image<float3, Host> CloseLoop::getAllVertex() const
{
    return _fusion->getAllVertex();
}
/*
bool CloseLoop::processKeyFrame()
{

    smoothNet->loadFrameData(_frame);
    bool found=smoothNet->findDescriptors(_frame);

    if(found)
    {
        int currPoseIdx=_isam->poseSize()-1;
        if(firstKeyFrame)
        {
            prevKeyPoseIdx=currPoseIdx;
            firstKeyFrame=false;
        }
        else
        {
            sMatrix4 tf=smoothNet->getTf();
            sMatrix6 cov=smoothNet->calculateCov();
            _isam->addPoseConstrain(prevKeyPoseIdx,currPoseIdx,tf,cov);
            optimize();
            reInit();
            prevKeyPoseIdx=0;
        }
    }
    smoothNet->clear();
    return true;
}
*/
bool CloseLoop::optimize()
{
    double err=_isam->optimize(_frame);

    std::cout<<"Optimization error:"<<err<<std::endl;
    if(err<params.optim_thr )
    {
        return false;
    }

#ifndef DISABLE_MAP_FIXES
    fixMap();
    _fusion->raycasting(_frame);
#endif
    return true;
}

void CloseLoop::removeOldNodes(int idx)
{
    auto depthIt=depths.begin();
    auto rgbIt=rgbs.begin();

    for(int i=0;i<idx;i++)
    {
        depthIt->release();
        rgbIt->release();

        depthIt++;
        rgbIt++;

        covars.pop_front();
        poses.pop_front();
        depths.pop_front();
        rgbs.pop_front();

        _isam->popFront();
    }
}

void CloseLoop::getIsamPoses(std::vector<sMatrix4> &vec)
{
    vec.clear();
    vec.reserve(_isam->poseSize());
    for(int i=0;i<_isam->poseSize();i++)
    {
        vec.push_back(_isam->getPose(i));
    }
}

void CloseLoop::fixMap()
{
    auto rdepthIt=depths.rbegin();
    auto rrgbIt=rgbs.rbegin();
    auto rposeIt=poses.rbegin();
    
    while(rposeIt!=poses.rend() )
    {
        _fusion->deIntegration(*rposeIt,*rdepthIt,*rrgbIt);
        rposeIt++;
        rdepthIt++;
        rrgbIt++;
    }
    
    auto depthIt=depths.begin();
    auto rgbIt=rgbs.begin();
    poses.clear();
    int i=0;
    while(depthIt!=depths.end() )
    {
        sMatrix4 newPose=_isam->getPose(i);
        _fusion->reIntegration(newPose,*depthIt,*rgbIt);
        poses.push_back(newPose);  
        depthIt++;
        rgbIt++;        
        i++;
    }

    rposeIt=poses.rbegin();
    _fusion->setPose(*rposeIt);
}

sMatrix4 CloseLoop::getPose() const
{
    return _fusion->getPose();
}

void CloseLoop::reInit(int idx)
{
    
}


void CloseLoop::reInit()
{
    /*
    int size=poses.size()-1;
    for(uint i=0; i<size;i++ )
    {
        DepthHost depth=depths[i];
        RgbHost rgb=rgbs[i];

        depth.release();
        rgb.release();
    }

    DepthHost initialDepth=depths[size];
    RgbHost initialRgb=rgbs[size];
    sMatrix4 initialPose=poses[size];

    depths.clear();
    rgbs.clear();
    poses.clear();
    _isam->clear();

    depths.push_back(initialDepth);
    rgbs.push_back(initialRgb);
    poses.push_back(initialPose);

    sMatrix6 cov;
    cov=cov*params.cov_small;
        
    _isam->init(initialPose,cov);
    covars.push_back(cov);
    */
}

void CloseLoop::clear()
{        
    auto depthIt=depths.begin();
    auto rgbIt=rgbs.begin();
//     auto covIt=covars.begin();
//     auto poseIt=poses.poses();
    
    while(depthIt!=depths.end() )
    {
        depthIt->release();
        rgbIt->release();
    }
    

    depths.clear();
    rgbs.clear();
    poses.clear();
    covars.clear();
    _isam->clear();
}

CloseLoop::~CloseLoop()
{
    clear();
    delete _fusion;
    delete _isam;
}
