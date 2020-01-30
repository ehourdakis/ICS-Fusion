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
        _isam->init(_fusion->getPose() );
        prevPose=_fusion->getPose();
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

bool CloseLoop::addPoseConstrain(const sMatrix4 &pose)
{
    _isam->addFixPose(pose);
    return true;
}

bool CloseLoop::optimize()
{
    double err=_isam->optimize(_frame);
    return true;
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
