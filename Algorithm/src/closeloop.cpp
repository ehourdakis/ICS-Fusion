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
//    smoothNet->readKeyPts();
    smoothNet->loadFrameData(_frame);
    smoothNet->findKeyPts(_frame);
    smoothNet->calculateLRF(_frame);
    smoothNet->callCnn(_frame);
    smoothNet->readDescriptorCsv();
    smoothNet->saveKeyPts(_frame);
    smoothNet->saveKeyVertex(_frame);

    sMatrix4 tr;
    float rmse;
    float fitness=smoothNet->findTransformation(tr,rmse,_frame);
    sMatrix6 cov;
    cov=cov*(rmse*rmse*fitness);
    std::cout<<"Registration fitness:"<<fitness<<std::endl;

    int currPose=_isam->poseSize()-1;
    if(fitness>0)
    {
        //_isam->addPoseConstrain(prevKeyPose,currPose,tr,cov);
        _isam->addPoseConstrain(currPose,prevKeyPose,tr,cov);
        optimize();
    }
    smoothNet->clear();
    prevKeyPose=currPose;

    return true;
}

bool CloseLoop::optimize()
{
    double err=_isam->optimize(_frame);
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

    /*
    clear();
    _isam->clear();
    _isam->init(finalPose);
    */
}

sMatrix4 CloseLoop::getPose() const
{
    return _fusion->getPose();
}

void CloseLoop::clear()
{        
    for(uint i=0; i<poses.size();i++ )
    {
        Host depth=depths[i];
        Host rgb=rgbs[i];

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
