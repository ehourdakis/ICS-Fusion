#include<mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3D.h>

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

#define SLAMBENCH_CH

static kparams_t params;
bool tracked ;
bool integrated;

slambench::io::pixelformat::EPixelFormat depthFormat;
SLAMBenchLibraryHelper *lib;

static int frame = 0;
static  slambench::TimeStamp frameTimeStamp;
std::string shader_dir;
uint3 *keypts1;

Matrix4 initialGt;

static IcsFusion * icsFusion=0;
static CloseLoop *loopCl=0;

static uchar3* inputRGB;
static uchar3* outputRGB;
static uchar3* outputGtRGB;
static uchar3* outputFeatRGB;
static uchar3* inputDepthRGB;
static uchar3* trackRGB;
static uint16_t* inputDepth;

static sMatrix4 gtPose;
// ===========================================================
// SLAMBench Sensors
// ===========================================================

static slambench::io::DepthSensor *depth_sensor;
static slambench::io::CameraSensor *rgb_sensor;

std::vector<sMatrix4> poses;
// ===========================================================
// SLAMBench Outputs
// ===========================================================

slambench::outputs::Output *pose_output;

slambench::outputs::Output *rgb_frame_output;
slambench::outputs::Output *track_frame_output;
slambench::outputs::Output *volume_frame_output;
slambench::outputs::Output *depth_frame_output;
slambench::outputs::Output *gt_frame_output;
slambench::outputs::Output *feat_frame_output;


// ===========================================================
// FunctionsDeclaration
// ===========================================================
sMatrix4 getGt(/*SLAMBenchLibraryHelper *lib*/ const slambench::TimeStamp &ts_p, slambench::outputs::BaseOutput *output);
sMatrix4 getGtTransformed(/*SLAMBenchLibraryHelper *lib*/ const slambench::TimeStamp &ts_p, slambench::outputs::BaseOutput *output);

// ===========================================================
// Functions Implementation
// ===========================================================

Eigen::Matrix4f sMatrix4ToEigen(const sMatrix4 &mat)
{
    Eigen::Matrix4f ret;
    for (int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            ret(i,j) = mat(i,j);
    return ret;
}

void savePoses(char *fileName)
{
    using namespace std;
    ofstream file(fileName, std::ios::out);

    for(uint p=0;p<poses.size();p++)
    {
        sMatrix4 pose=poses[p];
        Eigen::Matrix3f rot;

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

    Matrix4 poseMatrix;

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
    std::cout << "input size is = "<<params.inputSize.x<<","<<params.inputSize.y<<std::endl;

    loopCl=new CloseLoop(params,poseMatrix);

    pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);
    slam_settings->GetOutputManager().RegisterOutput(pose_output);

    rgb_frame_output = new slambench::outputs::Output("RGB Frame", slambench::values::VT_FRAME);
    rgb_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(rgb_frame_output);

    volume_frame_output = new slambench::outputs::Output("Volume Frame", slambench::values::VT_FRAME);
    volume_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(volume_frame_output);

    track_frame_output = new slambench::outputs::Output("track Frame", slambench::values::VT_FRAME);
    track_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(track_frame_output);

    gt_frame_output = new slambench::outputs::Output("Gt Frame", slambench::values::VT_FRAME);
    gt_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(gt_frame_output);

    feat_frame_output = new slambench::outputs::Output("Feature Frame", slambench::values::VT_FRAME);
    feat_frame_output->SetKeepOnlyMostRecent(true);
    slam_settings->GetOutputManager().RegisterOutput(feat_frame_output);

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

    //enable this to use ground trouth pose instead of calculating visual odometry.
    //This is usefull for map integration testing
#if 0
    gtPose=getGtTransformed(frameTimeStamp,slam_settings->getGt());
    tracked=loopCl->addFrameWithPose(inputDepth,inputRGB,gtPose);
#else
    tracked=loopCl->addFrame(inputDepth,inputRGB);
#endif

    IcsFusion *icsFusion=loopCl->fusion();
    
    //use ground truth pose for loop closure.
    //This is usefull for loop closure testing
#ifdef SLAMBENCH_CH
    if(frame==3)
    {
        initialGt=getGt(frameTimeStamp,slam_settings->getGt()); 
        std::cout<<"inital gt"<<std::endl;
        std::cout<<initialGt<<std::endl;
    }
    if(frame>=3 )
    {
        
        //poses.push_back(gtPose);
        if(loopCl->isKeyFrame()  )
        {
            gtPose=getGtTransformed(frameTimeStamp,slam_settings->getGt());
            std::cout<<"edo"<<std::endl;
            poses.push_back(gtPose);
//            loopCl->doLoopClosure(gtPose);

            char buf[32];
            sprintf(buf,"f_/f_%d_poses_gt",frame+1);
            savePoses(buf);
        }
    }
#endif

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

    delete inputDepthRGB;
    delete trackRGB;

    return true;
}

//get ground truth using the first ground truth value as origin 
sMatrix4 getGtTransformed(/*SLAMBenchLibraryHelper *lib*/ const slambench::TimeStamp &ts_p,slambench::outputs::BaseOutput *output)
{
    sMatrix4 gt=getGt(ts_p,output);
    sMatrix4 pose;

    pose.data[0].w+=params.volume_direction.x;
    pose.data[1].w+=params.volume_direction.y;
    pose.data[2].w+=params.volume_direction.z;
    
    sMatrix4 delta=inverse(initialGt)*gt;

    return pose*delta;
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


    /*
     sMatrix4 T_B_P;   
     T_B_P.data[0]=make_float4(0,-1,0,0);
     T_B_P.data[1]=make_float4(0,0,-1,0);
     T_B_P.data[2]=make_float4(1,0,0,0);
     T_B_P.data[3]=make_float4(0,0,0,1);
    */
   
//enable this for room dataset
#if 0
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


    if(gt_frame_output->IsActive() )
    {
        icsFusion->getImageProjection(gtPose,outputGtRGB);
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        gt_frame_output->AddPoint(ts, new slambench::values::FrameValue( params.inputSize.x,
                                                                         params.inputSize.y,
                                                                         slambench::io::pixelformat::RGB_III_888,
                                                                         outputGtRGB));
    }


    if(feat_frame_output->IsActive() )
    {
        loopCl->getFeatDetector()->getFeatImage(outputFeatRGB);
        icsFusion->getImageProjection(gtPose,outputGtRGB);
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        feat_frame_output->AddPoint(ts, new slambench::values::FrameValue( params.inputSize.x,
                                                                         params.inputSize.y,
                                                                         slambench::io::pixelformat::RGB_III_888,
                                                                         outputFeatRGB));
    }

    return true;
}
