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
     firstKeyFrame(true),
     prevKeyPoseIdx(-1),
     passedFromLastKeyFrame(-1)
{
    _fusion = new IcsFusion(params,initPose);


#ifdef USE_G2O
     _isam=new G2oGraph(params);
#else
    _isam=new Isam(params);
#endif

    harris=new Harris();

    _featDet=new FeatureDetector(p,_fusion,_isam);
    _keyMap=new keyptsMap(_isam,_fusion,params);
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
    //std::cout<<"[FRAME="<<_frame<<"]"<<std::endl;

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
         
        DepthHost rawDepth;
        _fusion->getDepthRaw(rawDepth);
        depths.push_back(rawDepth);

        RgbHost rawRgb;
        _fusion->getImageRaw(rawRgb);
        rgbs.push_back(rawRgb);

        covars.push_back(cov);
        poses.push_back(pose);
        
        passedFromLastKeyFrame=0;
    }
    else if(_frame>3 && tracked )
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
        //std::cout<<"ICP Fitness:"<<icpFitness<<std::endl;
        //std::cout<<"ICP cov:\n"<<icpCov<<std::endl;

        //icpCov=icpCov*1000*(1/icpFitness);

        //icpCov=icpCov*1e4;

        covars.push_back(icpCov);
        _isam->addFrame(pose,icpCov);

        passedFromLastKeyFrame++;
    }

    bool raycast=_fusion->raycasting(_frame);
    if(!raycast)
    {
        std::cerr<<"[FRAME="<<_frame<<"] Raycast faild!"<<std::endl;
    }

    return tracked;
}

void CloseLoop::saveKeyMap(char *descrFile,char *poitsFile,char *frameFile)
{
    _keyMap->saveMap(descrFile,poitsFile,frameFile);
}

void CloseLoop::showKeypts(uchar3 *out)
{
#ifdef DRAW_MATCHES
     std::vector<cv::DMatch> good_matches=_keyMap->goodMatches();
    _featDet->getFeatImage(out,good_matches);
#else
    _featDet->getFeatImage(out);
#endif
}

void CloseLoop::saveImage(char *filename)
{
    _featDet->saveImage(filename);
}

Image<float3, Host> CloseLoop::getAllVertex() const
{
    return _fusion->getAllVertex();
}

void CloseLoop::getMatches(std::vector<float3> &prevPts,
                std::vector<float3> &newPts)
{
    _keyMap->getMatches(lastKeyPts,prevPts,newPts);
}

bool CloseLoop::processKeyFrame()
{
    std::cout<<"[KEY FRAME="<<_frame<<"]"<<std::endl;

    auto rgb=rgbs.rbegin();
    auto depth=depths.rbegin();
    std::vector<float3> pts;
    std::vector<FeatDescriptor> descr;
    _featDet->detectFeatures(_frame,*depth,*rgb,pts,descr);

    if(pts.size()==0)
        return false;

    lastKeyPts=pts;
    lastDescr=descr;

    if(_keyMap->isEmpty() )
    {
        _keyMap->addKeypoints(lastKeyPts,lastDescr,passedFromLastKeyFrame);
        return true;

    }
    else
    {
        if(_keyMap->matching(lastKeyPts,lastDescr,_frame) )
        {
            std::cout<<"pre optimize"<<std::endl;
            bool b=optimize();
            std::cout<<"clear"<<std::endl;
            //clearFirsts(passedFromLastKeyFrame);
            reInit();
            std::cout<<"clear2"<<std::endl;
            
//             _keyMap->addKeypoints(lastKeyPts,lastDescr,0);
            prevKeyPoseIdx=0;
            passedFromLastKeyFrame=0;
            return b;
        }
    }

    
    return false;
}

void CloseLoop::clearFirsts(int idx)
{
    for(int i=0;i<idx;i++)
    {
        depths.front().release();
        rgbs.front().release();
        depths.pop_front();
        rgbs.pop_front();
        poses.pop_front();
        covars.pop_front();

        _isam->popFront();
    }

    _keyMap->clear();
}

void CloseLoop::saveDescriptors(char *fileName)
{    
    std::ofstream outFile(fileName, std::ios::out);
    for(int i=0;i<lastDescr.size();i++)
    {
        FeatDescriptor d=lastDescr[i];
        for(int j=0;j<d.size();j++)
        {
            outFile<<d.data[j]<<" ";
        }
        outFile<<d.s2<<"\n";   
    }
    outFile.close();
}

void CloseLoop::saveKeyPts(char *fileName)
{
    std::ofstream outFile(fileName, std::ios::out);
    for(int i=0;i<lastKeyPts.size();i++)
    {
        float3 pt=lastKeyPts[i];        
        outFile<<pt.x<<" "<<pt.y<<" "<<pt.z<<"\n";
    }
    outFile.close();
}

void CloseLoop::saveCorrespondance(char *fileName)
{
    std::vector<cv::DMatch> good_matches=_keyMap->goodMatches();
    if(good_matches.size()==0)
        return;
    
    std::ofstream outFile(fileName, std::ios::out);
    
    for(int i=0;i<good_matches.size();i++)
    {
        cv::DMatch m=good_matches[i];           
        outFile<<m.trainIdx<<" "<<m.queryIdx<<"\n";
    }
    outFile.close();
}


//void CloseLoop::clear()
//{

//}

bool CloseLoop::optimize()
{
    double err=_isam->optimize(_frame);

    std::cout<<"Optimization error:"<<err<<std::endl;
    if(err>params.optim_thr && false)
    {
        std::cout<<"Aborting optimization..."<<std::endl;
        return false;
    }

#ifndef DISABLE_MAP_FIXES
    fixMap();
    _fusion->raycasting(_frame);
#endif
    return true;
}

void CloseLoop::saveIcpCov(char *fileName) const
{
    if(!tracked)
        return ;

    std::ofstream outFile(fileName, std::ios::out);
    sMatrix6 cov=covars.back();

    for(int i=0;i<6;i++)
    {
        for(int j=0;j<6;j++)
        {
            outFile<<cov(i,j)<<" ";
        }
        outFile<<std::endl;
    }
    outFile.close();

}

void CloseLoop::saveDescrCov(char *fileName) const
{

    std::ofstream outFile(fileName, std::ios::out);

    for(int i=0;i<lastDescr.size();i++)
    {
        FeatDescriptor d=lastDescr[i];
        sMatrix3 cov=d.cov;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                outFile<<cov(i,j)<<" ";
            }
        }
        outFile<<std::endl;
    }
    outFile.close();
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
    std::cout<<"fixMap"<<std::endl;
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


    sMatrix4 kpose=_fusion->getPose();
    rposeIt=poses.rbegin();

//    sMatrix4 p=kpose- *rposeIt;
//    std::cout<<p<<std::endl;
    _fusion->setPose(*rposeIt);
//    std::cout<<"I:"<<rposeIt->get_translation()<<std::endl;
}

sMatrix4 CloseLoop::getPose() const
{
    return _fusion->getPose();
}

void CloseLoop::reInit()
{
    std::cout<<"reInit"<<std::endl;

    int size=poses.size()-1;
    auto depthIt=depths.begin();
    auto rgbIt=rgbs.begin();

    for(int i=0;i<depths.size()-1;i++)
    {
        depthIt->release();
        rgbIt->release();

        depthIt++;
        rgbIt++;
    }


    DepthHost initialDepth=*depths.rbegin();
    RgbHost initialRgb=*rgbs.rbegin();
    sMatrix4 initialPose=*poses.rbegin();

    depths.clear();
    rgbs.clear();
    poses.clear();
    _isam->clear();
    _keyMap->clear();

    depths.push_back(initialDepth);
    rgbs.push_back(initialRgb);
    poses.push_back(initialPose);

    sMatrix6 cov;
    cov=cov*params.cov_small;
        
    _isam->init(initialPose,cov);
    covars.push_back(cov);


    _keyMap->addKeypoints(lastKeyPts,lastDescr,0);
}

void CloseLoop::clear()
{        
    auto depthIt=depths.begin();
    auto rgbIt=rgbs.begin();
    
    while(depthIt!=depths.end() )
    {
        depthIt->release();
        rgbIt->release();
    }
    
    _isam->clear();
    _keyMap->clear();
    _keyMap->addKeypoints(lastKeyPts,lastDescr,_frame);

    depths.clear();
    rgbs.clear();
    poses.clear();
    covars.clear();

}

CloseLoop::~CloseLoop()
{
    clear();
    delete _fusion;
    delete _isam;
}
