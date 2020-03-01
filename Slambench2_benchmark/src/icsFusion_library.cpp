#include<icsFusion.h>
#include<kparams.h>
#include<volume.h>

#include <timings.h>
#include <SLAMBenchAPI.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/CameraSensorFinder.h>
#include <io/sensor/DepthSensor.h>
#include <io/SLAMFrame.h>

#include <fstream>

#include"closeloop.h"
#include<saveData.h>
#include <unistd.h>

#include<defs.h>

#define SLAMBENCH_CH

kparams_t params;

bool tracked ;
bool integrated;

sMatrix4 prevKeyPose;

slambench::io::pixelformat::EPixelFormat depthFormat;
SLAMBenchLibraryHelper *lib;

static int frame = 0;
static  slambench::TimeStamp frameTimeStamp;
std::string shader_dir;
uint3 *keypts1;

static sMatrix4 initialGt;

static IcsFusion * icsFusion=0;
static CloseLoop *loopCl=0;

static uchar3* inputRGB;
static uchar3* outputRGB;
static uchar3* outputGtRGB;
static uchar3* outputFeatRGB;
static uchar3* inputDepthRGB;
static uchar3* trackRGB;
static uint16_t* inputDepth;

static uchar3 *outputFeat;
// ===========================================================
// SLAMBench Sensors
// ===========================================================

static slambench::io::DepthSensor *depth_sensor;
static slambench::io::CameraSensor *rgb_sensor;

static std::vector<sMatrix4> gtPoses;
static std::vector<sMatrix4> kfusionPoses;
// ===========================================================
// SLAMBench Outputs
// ===========================================================

slambench::outputs::Output *pose_output;

slambench::outputs::Output *rgb_frame_output;
slambench::outputs::Output *track_frame_output;
slambench::outputs::Output *volume_frame_output;
slambench::outputs::Output *feat_frame_output;
slambench::outputs::Output *depth_frame_output;

// ===========================================================
// FunctionsDeclaration
// ===========================================================
sMatrix4 getGt(/*SLAMBenchLibraryHelper *lib*/ const slambench::TimeStamp &ts_p, slambench::outputs::BaseOutput *output);
sMatrix4 getGtTransformed(/*SLAMBenchLibraryHelper *lib*/ const slambench::TimeStamp &ts_p, slambench::outputs::BaseOutput *output);

// ===========================================================
// Functions Implementation
// ===========================================================

std::vector<float3> keyPts;
std::vector<float3> prevKeyPts;
int lastKeyFrame=-1;
sMatrix4 prevGt;


Eigen::Matrix4f sMatrix4ToEigen(const sMatrix4 &mat)
{
    Eigen::Matrix4f ret;
    for (int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            ret(i,j) = mat(i,j);
    return ret;
}

sMatrix4 homo(const Eigen::Vector3f &trans,const Eigen::Vector3f &rot)
{    
    sMatrix3  R_x;
    R_x(1,1)=cos(rot(0) );
    R_x(1,2)=-sin(rot(0) );
    R_x(2,1)=sin(rot(0) );
    R_x(2,2)=cos(rot(0) );
    
    sMatrix3  R_y;
    R_y(0,0)=cos(rot(1) );
    R_y(0,2)=sin(rot(1) );
    R_y(2,0)=-sin(rot(1) );
    R_y(2,2)=cos(rot(1) );
    
    sMatrix3  R_z;
    R_z(0,0)=cos(rot(2) );
    R_z(0,1)=-sin(rot(2) );
    R_z(1,0)=sin(rot(2) );
    R_z(1,1)=cos(rot(2) );
   
    sMatrix3 tmp=R_y*R_x;
    tmp=R_z*tmp;
 
    sMatrix4 ret;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            ret(i,j)=tmp(i,j);
    
    ret(0,3)=trans(0);
    ret(1,3)=trans(1);
    ret(2,3)=trans(2);
    ret(3,3)=1;
    return ret;
}

bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings)
{
    //TODO impement me
    (void)slam_settings;        
    return true;
}

bool sb_init_slam_system(SLAMBenchLibraryHelper * slam_settings)
{
    (void)slam_settings;
    /**
    * Retrieve RGB and Depth sensors,
    *  - check input_size are the same
    *  - check camera are the same
    *  - get input_file
    */

    setenv("SHADER_DIR", shader_dir.c_str(), 1);

    slambench::io::CameraSensorFinder sensor_finder;
    rgb_sensor = sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "rgb"}});
    depth_sensor = (slambench::io::DepthSensor*)sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "depth"}});

    if (rgb_sensor == nullptr)
    {
        std::cerr << "Invalid sensors found, RGB not found." << std::endl;
        return false;
    }
    if (depth_sensor == nullptr)
    {
        std::cerr << "Invalid sensors found, Depth not found." << std::endl;
        return false;
    }

    if(rgb_sensor->FrameFormat != slambench::io::frameformat::Raster) {
            std::cerr << "RGB data is in wrong format" << std::endl;
            return false;
    }
    if(depth_sensor->FrameFormat != slambench::io::frameformat::Raster) {
            std::cerr << "Depth data is in wrong format" << std::endl;
            return false;
    }
    if(rgb_sensor->PixelFormat != slambench::io::pixelformat::RGB_III_888) {
            std::cerr << "RGB data is in wrong format pixel" << std::endl;
            return false;
    }
    if(depth_sensor->PixelFormat != slambench::io::pixelformat::D_I_16) {
            std::cerr << "Depth data is in wrong pixel format" << std::endl;
            return false;
    }

    assert(depth_sensor->Width == rgb_sensor->Width);
    assert(depth_sensor->Height == rgb_sensor->Height);

    params.inputSize = make_uint2(rgb_sensor->Width, rgb_sensor->Height);
    params.computationSize = make_uint2(
                params.inputSize.x / params.compute_size_ratio,
                params.inputSize.y / params.compute_size_ratio);

    sMatrix4 poseMatrix;

    poseMatrix.data[0].w +=  params.volume_direction.x;
    poseMatrix.data[1].w +=  params.volume_direction.y;
    poseMatrix.data[2].w +=  params.volume_direction.z;

    params.camera =  make_float4(
			depth_sensor->Intrinsics[0],
			depth_sensor->Intrinsics[1],
			depth_sensor->Intrinsics[2],
			depth_sensor->Intrinsics[3]);

 	params.camera.x = params.camera.x *  params.computationSize.x;
 	params.camera.y = params.camera.y *  params.computationSize.y;
 	params.camera.z = params.camera.z *  params.computationSize.x;
 	params.camera.w = params.camera.w *  params.computationSize.y;

    std::cerr << "camera is = "
              << params.camera.x << ","
              << params.camera.y << ","
              << params.camera.z << ","
              << params.camera.w << std::endl;

    inputRGB = new uchar3[params.inputSize.x * params.inputSize.y];
    inputDepth = new uint16_t[params.inputSize.x * params.inputSize.y];
    outputRGB = new uchar3[params.inputSize.x * params.inputSize.y];
    outputGtRGB = new uchar3[params.inputSize.x * params.inputSize.y];
    outputFeatRGB= new uchar3[params.inputSize.x * params.inputSize.y];
    inputDepthRGB = new uchar3[params.inputSize.x * params.inputSize.y];
    trackRGB = new uchar3[params.inputSize.x * params.inputSize.y];

#ifdef DRAW_MATCHES
    outputFeat = new uchar3[params.inputSize.x * params.inputSize.y*2]; 
#else
    outputFeat = new uchar3[params.inputSize.x * params.inputSize.y]; 
#endif

    std::cout << "input size is = "<<params.inputSize.x<<","<<params.inputSize.y<<std::endl;

    loopCl=new CloseLoop(params,poseMatrix);

    pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);
    slam_settings->GetOutputManager().RegisterOutput(pose_output);

    rgb_frame_output = new slambench::outputs::Output("RGB Frame", slambench::values::VT_FRAME);
    rgb_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(rgb_frame_output);

    volume_frame_output = new slambench::outputs::Output("Volume Frame", slambench::values::VT_FRAME);
    feat_frame_output = new slambench::outputs::Output("Feature Frame", slambench::values::VT_FRAME);
    
    volume_frame_output->SetKeepOnlyMostRecent(true);
    feat_frame_output->SetKeepOnlyMostRecent(true);
    
    slam_settings->GetOutputManager().RegisterOutput(volume_frame_output);
    slam_settings->GetOutputManager().RegisterOutput(feat_frame_output);
    

    track_frame_output = new slambench::outputs::Output("track Frame", slambench::values::VT_FRAME);
    track_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(track_frame_output);

    depth_frame_output = new slambench::outputs::Output("Depth", slambench::values::VT_FRAME);
    depth_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(depth_frame_output);

    return true;
}


bool depth_ready = false;
bool rgb_ready   = false;

bool sb_update_frame (SLAMBenchLibraryHelper * slam_settings, slambench::io::SLAMFrame* s)
{
    (void)slam_settings;
    
    if (depth_ready && rgb_ready)
    {
        depth_ready = false;
        rgb_ready   = false;
    }

    assert(s != nullptr);

    char *target = nullptr;
    frameTimeStamp=s->Timestamp;

    if(s->FrameSensor == depth_sensor)
    {
        target = (char*)inputDepth;
        depth_ready = true;
        depthFormat=depth_sensor->PixelFormat;
    }
    else if(s->FrameSensor == rgb_sensor)
    {
        target = (char*)inputRGB;
        rgb_ready = true;
    }

    if(target != nullptr)
    {
        memcpy(target, s->GetData(), s->GetSize());
        s->FreeData();
    }

    return depth_ready && rgb_ready;
}

bool sb_process_once (SLAMBenchLibraryHelper * slam_settings)
{
    (void)slam_settings;

    if(frame==0)
        sleep(1);
    //enable this to use ground trouth pose instead of calculating visual odometry.
    //This is usefull for map integration testing
#if 0
    sMatrix4 gtPose=getGtTransformed(frameTimeStamp,slam_settings->getGt());
    tracked=loopCl->addFrameWithPose(inputDepth,inputRGB,gtPose);
#else
    loopCl->preprocess(inputDepth,inputRGB);
    tracked=loopCl->processFrame();
    kfusionPoses.push_back(loopCl->getPose());
#endif
    
    IcsFusion *icsFusion=loopCl->fusion();
    
    //use ground truth pose for loop closure.
    //This is usefull for loop closure testing
    sMatrix4 gtPose;
#ifdef SLAMBENCH_CH
    if(frame==3)
    {
        initialGt=getGt(frameTimeStamp,slam_settings->getGt()); 
        std::cout<<"inital gt"<<std::endl;
        std::cout<<initialGt<<std::endl;
        /*
        char buf[32];
        sprintf(buf,"data/ply/f_%d_vertices.ply",frame);
        Image<float3, Host> vert=icsFusion->getAllVertex();
        saveVertexPly(buf,vert);
        */
        
    }
    else if(frame>=3)
    {
        gtPose=getGtTransformed(frameTimeStamp,slam_settings->getGt());
#if 0

        gtPoses.push_back(gtPose);
        char buf[32];

        sprintf(buf,"data/gt/pose%d",frame);
        savePoseMat(buf,gtPose);
        
        sMatrix4 pose=loopCl->getPose();
        sprintf(buf,"data/poses/pose%d",frame);
        savePoseMat(buf,pose);


        sprintf(buf,"data/vert/vert%d.txt",frame);
        Image<float3, Host> vert=icsFusion->getAllVertex();
        saveVertex(buf,vert);
        vert.release();

        sprintf(buf,"data/norm/norm%d.txt",frame);
        Image<float3, Host> norm=icsFusion->getAllNormals();
        saveVertex(buf,norm);
        norm.release();

        sprintf(buf,"data/poses/cov%d.txt",frame);
        loopCl->saveIcpCov(buf);
#endif
    }

#endif

    bool _isKeyFrame=false;

#ifdef LOOP_CLOSURE_RATE

    if(frame==3)
        _isKeyFrame=true;
    if( frame>0 && (frame%LOOP_CLOSURE_RATE)==0)
        _isKeyFrame=true;
#endif
    if(_isKeyFrame)
    {

        double outlierTH = 0.01;

        loopCl->processKeyFrame();
        loopCl->showKeypts(outputFeat);
        prevKeyPts=keyPts;
        keyPts=loopCl->getKeypts();

        if(lastKeyFrame>0)
        {
            sMatrix4 delta=inverse(prevGt)*gtPose;
            std::cout<<delta<<std::endl;
                
            int outlierNum=0;
            std::vector<cv::DMatch> good_matches=loopCl->getKeyMap()->goodMatches();
            for(int i=0;i<good_matches.size();i++)
            {
                cv::DMatch m=good_matches[i];
                float3 v1=prevKeyPts[m.trainIdx];
                float3 v2=keyPts[m.queryIdx];

                v1=prevGt*v1;
                v2=gtPose*v2;
                

                
                //v2=prevGt*v2;

                float3 diff=make_float3(fabs(v1.x-v2.x),
                                        fabs(v1.y-v2.y),
                                        fabs(v1.z-v2.z) );

//                std::cout<<prevKeyPts[m.trainIdx]<<std::endl;
                if(diff.x>outlierTH || diff.y>outlierTH ||diff.z>outlierTH )
                {
//                    std::cout<<diff<<std::endl;
                    outlierNum++;
                }
            }
            std::cout<<"Number of outlier:"<<outlierNum<<std::endl;
        }

#if 0
        char buf[64];
        sprintf(buf,"data/feat/frame%d.png",frame);
        loopCl->saveImage(buf);
        
        sprintf(buf,"data/feat/descr%d.txt",frame);
        loopCl->saveDescriptors(buf);
        
        sprintf(buf,"data/feat/points%d.txt",frame);
        loopCl->saveKeyPts(buf);
        
        sprintf(buf,"data/feat/cov%d.txt",frame);
        loopCl->saveDescrCov(buf);

        if(lastKeyFrame>0)
        {
            sprintf(buf,"data/feat/corr_from_%d_to%d.txt",lastKeyFrame, frame);
            loopCl->saveCorrespondance(buf);
        }
#endif        
        lastKeyFrame=frame;
        prevGt=gtPose;

#ifdef DRAW_MATCHES
//        loopCl->reInit();
#endif

    }

    frame++;

    return true;
}

 
bool sb_get_tracked  (bool* tracking)
{
    *tracking = tracked;
    return true;
}

bool sb_clean_slam_system()
{
    IcsFusion *icsFusion=loopCl->fusion();
    Volume v=icsFusion->getVolume();
    saveVoxelsToFile(v,params, "voxels");
    
    std::cout<<"Cleaning"<<std::endl;
    delete loopCl;
    delete inputRGB;
    delete inputDepth;
    delete outputRGB;
    delete outputGtRGB;
    delete outputFeatRGB;
    delete outputFeat;

    delete inputDepthRGB;
    delete trackRGB;

    return true;
}

//get ground truth using the first ground truth value as origin 
sMatrix4 getGtTransformed(/*SLAMBenchLibraryHelper *lib*/ const slambench::TimeStamp &ts_p,slambench::outputs::BaseOutput *output)
{
    sMatrix4 gt=getGt(ts_p,output);        
    sMatrix4 delta=inverse(initialGt)*gt;

    return delta;
}

sMatrix4 getGtToOrigin(/*SLAMBenchLibraryHelper *lib*/ const slambench::TimeStamp &ts_p,slambench::outputs::BaseOutput *output)
{
    sMatrix4 pose;
    pose.data[0].w+=params.volume_direction.x;
    pose.data[1].w+=params.volume_direction.y;
    pose.data[2].w+=params.volume_direction.z;
    return pose*getGtTransformed(ts_p,output);
}

//Some extremly compicated macaroni code
sMatrix4 getGt(/*SLAMBenchLibraryHelper *lib*/ const slambench::TimeStamp &ts_p, slambench::outputs::BaseOutput *output)
{
    sMatrix4 ret;

    auto gt_traj=output->GetValues();

    if(gt_traj.size() == 0)
    {
        std::cerr << "**** Error: Empty GT." << std::endl;
        return ret;
    }

    auto next=gt_traj.begin();
    auto prev=next;
    while(next!=gt_traj.end() && next->first < ts_p )
    {
        prev=next;
        next++;
    }

    const slambench::values::Value *closest;
    if(next==gt_traj.end())
    {
        closest=prev->second;
    }
    else
    {
        if( (ts_p-prev->first) < (next->first-ts_p) )
        {
            closest=prev->second;
        }
        else
        {
            closest=next->second;
        }
    }

    //WHY??
    slambench::values::Value *vv=const_cast<slambench::values::Value*>(closest);

    const slambench::values::PoseValue *v=dynamic_cast< slambench::values::PoseValue* >(vv);
    if(v==0)
    {
        std::cerr<<"Wrong casting"<<std::endl;
        return ret;
    }
    const Eigen::Matrix4f& gt_pose = v->GetValue();
    for (int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            ret(i,j) = gt_pose(i,j);


//enable this for room dataset
#if 1
     float tmp=ret.data[0].y;
     ret.data[0].y=-ret.data[1].x;
     ret.data[1].x=-tmp;
     
     tmp=ret.data[1].z;
     ret.data[1].z=-ret.data[2].y;
     ret.data[2].y=-tmp;
#endif

    return ret;
}

bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *ts_p)
{

    IcsFusion *icsFusion=loopCl->fusion();

    slambench::TimeStamp ts = *ts_p;
    if(pose_output->IsActive())
    {        
        sMatrix4 pose=loopCl->getPose();
        Eigen::Matrix4f mat=sMatrix4ToEigen(pose);


        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        pose_output->AddPoint(ts, new slambench::values::PoseValue(mat));
    }


    if(rgb_frame_output->IsActive() && rgb_ready)
    {
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        rgb_frame_output->AddPoint(ts, new slambench::values::FrameValue( params.inputSize.x,
                                                                          params.inputSize.y,
                                                                          slambench::io::pixelformat::RGB_III_888,
                                                                          inputRGB));
    }


    if(depth_frame_output->IsActive() )
    {
        icsFusion->renderDepth(inputDepthRGB);
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        depth_frame_output->AddPoint(ts, new slambench::values::FrameValue(params.inputSize.x,
                                                                           params.inputSize.y,
                                                                           slambench::io::pixelformat::RGB_III_888,
                                                                           inputDepthRGB));
    }

    if(track_frame_output->IsActive()  )
    {
        icsFusion->renderTrack(trackRGB);
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        track_frame_output->AddPoint(ts, new slambench::values::FrameValue(params.inputSize.x,
                                                                           params.inputSize.y,
                                                                           slambench::io::pixelformat::RGB_III_888,
                                                                           trackRGB));
    }

    if(volume_frame_output->IsActive() )
    {        
        icsFusion->renderImage(outputRGB);
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        volume_frame_output->AddPoint(ts, new slambench::values::FrameValue( params.inputSize.x,
                                                                             params.inputSize.y,
                                                                             slambench::io::pixelformat::RGB_III_888,
                                                                             outputRGB));
    }

    if(feat_frame_output->IsActive() )
    {        
        
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        
#ifdef DRAW_MATCHES
        int size_x= params.inputSize.x*2;
        int size_y= params.inputSize.y;
#else
        int size_x= params.inputSize.x;
        int size_y= params.inputSize.y;
#endif
        feat_frame_output->AddPoint(ts, new slambench::values::FrameValue( size_x,
                                                                             size_y,
                                                                             slambench::io::pixelformat::RGB_III_888,
                                                                             outputFeat));
    }


    return true;
}
