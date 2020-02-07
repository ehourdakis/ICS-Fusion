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
     _frame(-1)
{
    _fusion = new IcsFusion(params,initPose);

#ifdef USE_G2O
    _isam=new G2oGraph(params);
#else
    _isam=new Isam();
#endif
    firstPose=initPose;

    smoothNet=new SmoothNet(_fusion,params);
    smoothNet->socketConnect();
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
    _frame++;
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
        sMatrix4 pose=_fusion->getPose();
        _isam->init(pose);
        prevPose=_fusion->getPose();
         
        DepthHost rawDepth;
        _fusion->getDepthRaw(rawDepth);
        depths.push_back(rawDepth);

        RgbHost rawRgb;
        _fusion->getImageRaw(rawRgb);
        rgbs.push_back(rawRgb);

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
        float icpFitness=_fusion->getFitness();
        std::cout<<"ICP Fitness:"<<icpFitness<<std::endl;
        //icpCov=icpCov*(1/icpFitness);
//        icpCov=icpCov*1000;
        std::cout<<"ICP Fitness:"<<icpCov<<std::endl;

        _isam->addFrame(pose,icpCov);
    }

    bool raycast=_fusion->raycasting(_frame);
    if(!raycast)
    {
        std::cerr<<"[FRAME="<<_frame<<"] Raycast faild!"<<std::endl;
    }

    return tracked;
}

bool CloseLoop::processKeyFrame()
{
    smoothNet->loadFrameData(_frame);
    sMatrix4 tr;
    float rmse,fitness;
    int currPose=_isam->poseSize()-1;
    bool optimized=false;
    sMatrix6 cov;
    if(smoothNet->findTf(tr,fitness,rmse,cov,_frame) )
    {
        std::cout<<"Registration fitness:"<<fitness<<std::endl;
        std::cout<<"Registration RMSE:"<<rmse<<std::endl;

//        sMatrix6 cov;
//        cov=cov*(rmse/fitness);
//        cov=cov*0.01;
        _isam->addPoseConstrain(prevKeyPose,currPose,tr,cov);
        std::cout<<cov<<std::endl;
        optimized=optimize();
//        if(optimized)
//        {
//            reInit();
//        }
    }
//    if(optimized)
//    {
//        prevKeyPose=currPose;
//    }

    prevKeyPose=currPose;
    smoothNet->clear();


    return true;
}

bool CloseLoop::optimize()
{
    double err=_isam->optimize(_frame);
    std::cout<<"Optimization error:"<<err<<std::endl;
    if(err<params.optim_thr )
    {
        fixMap();
        bool raycast=_fusion->raycasting(_frame);
    }
    return true;
}

void CloseLoop::fixMap()
{
    for(int i=(int)poses.size()-1; i>=0; i--)
    {
        _fusion->deIntegration(poses[i],depths[i],rgbs[i]);
    }
    poses.clear();
    
    for(int i=0;i<_isam->poseSize();i++)
    {
        sMatrix4 newPose=_isam->getPose(i);
        _fusion->reIntegration(newPose,depths[i],rgbs[i]);
        poses.push_back(newPose);        
    }
    sMatrix4 finalPose=_isam->getPose(poses.size()-1);
    _fusion->setPose(finalPose);
}

sMatrix4 CloseLoop::getPose() const
{
    return _fusion->getPose();
}

void CloseLoop::reInit()
{
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

    _isam->init(initialPose);
    prevKeyPose=0;
}

void CloseLoop::clear()
{        
    int size=poses.size();
    for(uint i=0; i<size;i++ )
    {
        DepthHost depth=depths[i];
        RgbHost rgb=rgbs[i];

        depth.release();
        rgb.release();
    }

    depths.clear();
    rgbs.clear();
    poses.clear();
    _isam->clear();
}

CloseLoop::~CloseLoop()
{
    clear();
    delete _fusion;
    delete _isam;
}
