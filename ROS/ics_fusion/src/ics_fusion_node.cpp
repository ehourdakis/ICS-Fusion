#include <ros/ros.h>

//#include<swap>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <string.h>
#include <kernels.h>

#include <tf/LinearMath/Matrix3x3.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <actionlib/client/simple_action_client.h>
#include <smoothnet_3d/SmoothNet3dAction.h>

#include<closeloop.h>
#include<kparams.h>
#include<icsFusion.h>
#include<volume.h>

#include<defs.h>

#include <opencv2/core/core.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_srvs/SetBool.h>

#define CAM_INFO_TOPIC "/camera/depth/camera_info"
#define RGB_TOPIC "/camera/rgb/image_rect_color"
#define DEPTH_TOPIC "/camera/depth/image_rect"
#define GEM_LEFT_TOPIC "/gem/LLegContactProbability"
#define GEM_RIGHT_TOPIC "/gem/RLegContactProbability"


#define PUB_VOLUME_TOPIC "/ics_fusion/volume_rendered"
#define PUB_ODOM_TOPIC "/ics_fusion/odom"
#define PUB_POINTS_TOPIC "/ics_fusion/pointCloud"
#define PUB_IMAGE_TOPIC "/ics_fusion/volume_rgb"
#define PUB_KEY_FRAME_TOPIC "/ics_fusion/key_frame"
#define PUB_HARRIS_FRAME_TOPIC "/ics_fusion/harris"

#define SMOOTHNET_SERVER "smoothnet_3d"

#define DEPTH_FRAME "camera_rgb_optical_frame"
// #define VO_FRAME "visual_odom"
#define VO_FRAME "odom"
#define ODOM_FRAME "odom"
#define BASE_LINK "base_link"

#define FEET_PROB_THR 0.95

#define PUBLISH_POINT_RATE 10
#define PUBLISH_IMAGE_RATE 1

ros::ServiceClient bagClient;


typedef unsigned char uchar;


kparams_t params;
CloseLoop *loopCl=nullptr;
int frame = 0;

//Buffers
uint16_t *inputDepth=0;
float *inputDepthFl=0;
uchar3 *inputRGB;
uchar3 *volumeRender;

//other params
bool publish_volume=true;
bool publish_points=true;
bool publish_key_frame=true;

bool publish_key_points = true;

int publish_points_rate;
int key_frame_thr;
int keypt_size;

//ros publishers
ros::Publisher volume_pub;
ros::Publisher odom_pub ;
ros::Publisher points_pub;
ros::Publisher key_frame_pub;
ros::Publisher harris_pub;
ros::Publisher pcl_pub0, pcl_pub1;
sensor_msgs::PointCloud pcl_msg0, pcl_msg1;
//frames
std::string depth_frame,vo_frame,base_link_frame,odom_frame;

bool keyFrameProcessing=false;
//functions
void publishVolumeProjection();
void publishOdom();
void publishPoints();

int leftFeetValue=0;
int rightFeetValue=0;
int doubleSupport=0;
int passedFromLastKeyFrame=0;

//keypts vertex
std::vector<float3> keyVert;
std::vector<float3> prevKeyVert;

//poses
sMatrix4 keyFramePose;
sMatrix4 prevKeyFramePose;

//visualization data
uchar3 *featImage;

smoothnet_3d::SmoothNet3dResult snResult;

actionlib::SimpleActionClient<smoothnet_3d::SmoothNet3dAction> *smoothnetServer=0;

geometry_msgs::Pose transform2pose(const geometry_msgs::Transform &trans);

#ifdef PUBLISH_ODOM_PATH

#define PUB_ODOM_PATH_TOPIC "/ics_fusion/odom_path"
nav_msgs::Path odomPath, isamPath;
ros::Publisher odom_path_pub ;
void publishOdomPath(geometry_msgs::Pose &p);

#endif

#ifdef PUBLISH_ISAM_PATH

#define PUB_ISAM_PATH_TOPIC "/ics_fusion/isam_path"
ros::Publisher isam_path_pub;
void publishIsamPath();

#endif

void publishHarris();
void publishKeyPoints();

void stopBag();
void contBag();

geometry_msgs::Pose transform2pose(const geometry_msgs::Transform &trans)
{
    geometry_msgs::Pose pose;
    pose.position.x=trans.translation.x;
    pose.position.y=trans.translation.y;
    pose.position.z=trans.translation.z;

    pose.orientation.x=trans.rotation.x;
    pose.orientation.y=trans.rotation.y;
    pose.orientation.z=trans.rotation.z;
    pose.orientation.w=trans.rotation.w;

    return pose;
}

void readIteFusionParams(ros::NodeHandle &n_p)
{   
    kparams_t par;
    /*
    n_p.getParam("compute_size_ratio",par.compute_size_ratio);
    n_p.getParam("integration_rate",par.integration_rate);
    n_p.getParam("rendering_rate",par.rendering_rate);
    n_p.getParam("tracking_rate",par.tracking_rate);
    
    std::vector<int> vol_res;
    n_p.getParam("volume_resolution",vol_res);
    par.volume_resolution = make_uint3((uint)vol_res[0],(uint)vol_res[1],(uint)vol_res[2]);    
    
    std::vector<float> vol_direct;
    n_p.getParam("volume_direction",vol_direct);
    par.volume_direction = make_float3(vol_direct[0],vol_direct[1],vol_direct[2]);    
    
    std::vector<float> vol_size;
    n_p.getParam("volume_size",vol_size);
    par.volume_size = make_float3(vol_size[0],vol_size[1],vol_size[2]);    
    
    n_p.getParam("pyramid",par.pyramid);  
    
    n_p.getParam("mu",par.mu);
    n_p.getParam("icp_threshold",par.icp_threshold);
    */
    params=par;
}

void doLoopClosure()
{
    stopBag();
    passedFromLastKeyFrame=0;
    loopCl->processKeyFrame();

    prevKeyFramePose=keyFramePose;
    keyFramePose=loopCl->getPose();

    if(publish_key_points &&false)
    {
        publishKeyPoints();
    }

   // std::cout<<inverse(prevKeyFramePose)*keyFramePose<<std::endl;

    publishHarris();  
    contBag();
}

void publishKeyPoints()
{
    std::vector<float3> prevKeyVert;
    std::vector<float3> keyVert;

    loopCl->getMatches(prevKeyVert,keyVert);

    pcl_msg0.points.resize(prevKeyVert.size());
    pcl_msg1.points.resize(keyVert.size());
    int i = 0;
    float3 v;

    while(i<max(prevKeyVert.size(),keyVert.size()))
    {
        if(i<prevKeyVert.size())
        {
            v=prevKeyVert[i];
            v=prevKeyFramePose*v;
            v=prevKeyVert[i];
            v.x-=params.volume_direction.x;
            v.y-=params.volume_direction.y;
            v.z-=params.volume_direction.z;

            v=fromVisionCordV(v);

            pcl_msg0.points[i].x = v.x;
            pcl_msg0.points[i].y = v.y;
            pcl_msg0.points[i].z = v.z;
        }

        if(i<keyVert.size())
        {
            v=keyVert[i];
            v=keyFramePose*v;
            v=keyVert[i];
            v.x-=params.volume_direction.x;
            v.y-=params.volume_direction.y;
            v.z-=params.volume_direction.z;

            v=fromVisionCordV(v);

            pcl_msg1.points[i].x = v.x;
            pcl_msg1.points[i].y = v.y;
            pcl_msg1.points[i].z = v.z;
        }
        i++;
    }
    pcl_msg0.header.frame_id=VO_FRAME;
    pcl_msg1.header.frame_id=VO_FRAME;

    pcl_msg0.header.stamp=ros::Time::now();
    pcl_msg1.header.stamp=ros::Time::now();

    pcl_pub1.publish(pcl_msg1);
    pcl_pub0.publish(pcl_msg0);
}

void smoothnetResultCb(const actionlib::SimpleClientGoalState &state,
            const smoothnet_3d::SmoothNet3dResultConstPtr& results)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    keyFrameProcessing=false;
    snResult=*results;
    return;
}

bool hasStableContact()
{
    int leftFeetVal=leftFeetValue;
    int rightFeetVal=rightFeetValue;
    
    bool ret=false;
    if(passedFromLastKeyFrame>key_frame_thr)
    {
        if( (rightFeetVal>2) && (leftFeetVal<-2) )
            ret=true;
        else if( (rightFeetVal<-2) && (leftFeetVal>2))
            ret=true;
//        else if(doubleSupport>3)
//            ret=true;
    }
    return ret;
}

bool isKeyFrame()
{
#ifdef DISABLE_KEY_FRAMES
    return false;
#endif
    
    if(keyFrameProcessing)
        return false;
    
    return hasStableContact();
}

void imageAndDepthCallback(const sensor_msgs::ImageConstPtr &rgb,const sensor_msgs::ImageConstPtr &depth)
{    
    passedFromLastKeyFrame++;
    if(strcmp(rgb->encoding.c_str(), "rgb8")==0) //rgb8
    {
        memcpy(inputRGB,rgb->data.data(),params.inputSize.y*params.inputSize.x*sizeof(uchar)*3 );        
    }
    else
    {
        ROS_ERROR("Not supported rgb format.");
        return;
    }
    
    if(strcmp(depth->encoding.c_str(), "32FC1")==0) //32FC1
    {
        if(inputDepthFl==0)
            inputDepthFl=new float[params.inputSize.x * params.inputSize.y];
            
        memcpy(inputDepthFl,depth->data.data(),params.inputSize.y*params.inputSize.x*sizeof(float) );
        loopCl->preprocess(inputDepthFl,inputRGB);
    }
    else if(strcmp(depth->encoding.c_str(), "16UC1")==0) //16UC1
    {
        if(inputDepth==0)
            inputDepth = new uint16_t[params.inputSize.x * params.inputSize.y];   
        
        memcpy(inputDepth,depth->data.data(),params.inputSize.y*params.inputSize.x*2);
        loopCl->preprocess(inputDepth,inputRGB);
    }
    else
    {
        ROS_ERROR("Not supported depth format.");
        return;
    }
    

    loopCl->processFrame();
    bool _isKeyFrame=false;
#ifndef DISABLE_LOOP_CLOSURE

  #ifdef LOOP_CLOSURE_RATE
    if(frame==4)
        _isKeyFrame=true;
    if( frame>0 && (frame%LOOP_CLOSURE_RATE)==0)
        _isKeyFrame=true;
  #else
    _isKeyFrame=isKeyFrame();
  #endif

#ifdef KEY_FRAME_2
    _isKeyFrame=false;
    if(frame==KEY_FRAME_1)
        _isKeyFrame=true;
    else if(frame==KEY_FRAME_2)
        _isKeyFrame=true;
    else
        _isKeyFrame=false;
#endif

#endif    
    
    if(_isKeyFrame)
    {
         doLoopClosure();
         #ifdef PUBLISH_ISAM_PATH
            publishIsamPath();
        #endif
    }

    if(publish_key_frame)
    {
        std_msgs::Float32 data;
        data.data=(float)_isKeyFrame;
        key_frame_pub.publish(data);
    }


    publishOdom();
    

    
    if(publish_volume)
    {
        publishVolumeProjection();
    }
    
    
    if(publish_points && frame % publish_points_rate ==0)
         publishPoints();
    
    frame++;
}

void camInfoCallback(sensor_msgs::CameraInfoConstPtr msg)
{
    params.camera =  make_float4(msg->K[0],msg->K[4],msg->K[2],msg->K[5]);
    params.inputSize.y=msg->height;
    params.inputSize.x=msg->width;
    
    params.computationSize = make_uint2(
                params.inputSize.x / params.compute_size_ratio,
                params.inputSize.y / params.compute_size_ratio);

    
    ROS_INFO("camera is = %f, %f, %f, %f",
             params.camera.x,
             params.camera.y,
             params.camera.z,
             params.camera.w);
    
    sMatrix4 poseMatrix;  
    poseMatrix(0,3)=params.volume_direction.x;
    poseMatrix(1,3)=params.volume_direction.y;
    poseMatrix(2,3)=params.volume_direction.z;
  

    inputRGB     = new uchar3[params.inputSize.x * params.inputSize.y];    
    volumeRender = new uchar3[params.computationSize.x * params.computationSize.y];
#ifdef DRAW_MATCHES
    featImage = new uchar3[params.inputSize.x * params.inputSize.y*2]; 
#else
    featImage = new uchar3[params.inputSize.x * params.inputSize.y]; 
#endif
    
    loopCl=new CloseLoop(params,poseMatrix);
}

void publishVolumeProjection()
{
    IcsFusion *icsFusion=loopCl->fusion();
    icsFusion->renderImage(volumeRender);
    
    sensor_msgs::Image image;
    image.header.stamp=ros::Time::now();

    image.width=params.inputSize.x;
    image.height=params.inputSize.y;
    
    int step_size=sizeof(uchar)*3;
    image.is_bigendian=0;
    image.step=step_size*image.width;
    image.header.frame_id=std::string("IcsFusion_volume");
    image.encoding=std::string("rgb8");

    uchar *ptr=(uchar*)volumeRender;
    image.data=std::vector<uchar>(ptr ,ptr+(params.computationSize.x * params.computationSize.y*step_size) );
    volume_pub.publish(image);
}

void publishHarris()
{
//     cv::Mat mat;
    loopCl->showKeypts(featImage);
    sensor_msgs::Image image;
    image.header.stamp=ros::Time::now();
    
#ifdef DRAW_MATCHES
    image.width=params.inputSize.x*2;
    image.height=params.inputSize.y;
#else 
    image.width=params.inputSize.x;
    image.height=params.inputSize.y;
#endif   
    
#ifdef DRAW_MATCHES
    int step_size=sizeof(uchar)*3;
#else
    int step_size=sizeof(uchar)*3;
#endif
    image.is_bigendian=0;
    image.step=step_size*image.width;
    image.header.frame_id=std::string("IcsFusion_volume");
    image.encoding=std::string("rgb8");
    uchar *ptr=(uchar*)featImage;
    image.data=std::vector<uchar>(ptr ,ptr+(image.width*image.height*step_size) );
    /*
    cv_bridge::CvImage out_msg;

    out_msg.image=mat;

    out_msg.header.frame_id=std::string("IcsFusion_volume");
//    out_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    //out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    //out_msg.image    = mat; // Your cv::Mat


    //out_msg.encoding = "8UC1";
    //out_msg.encoding = "32FC1";
    out_msg.encoding = "rgb8";
//    CV_8UC3

*/
    harris_pub.publish(image);
}

void publishOdom()
{
    sMatrix4 pose = loopCl->getPose();

    //
    pose(0,3)-=params.volume_direction.x;
    pose(1,3)-=params.volume_direction.y;
    pose(2,3)-=params.volume_direction.z;
//     pose=inverse(pose);
    pose=fromVisionCord(pose);
    /*
    tf::Vector3 vec[3];
    for(int i=0;i<3;i++)
    {
        vec[i]=tf::Vector3(pose.data[i].x,pose.data[i].y,pose.data[i].z);
    }    
    */
    tf::Matrix3x3 rot_matrix( pose(0,0),pose(0,1),pose(0,2),
                              pose(1,0),pose(1,1),pose(1,2),
                              pose(2,0),pose(2,1),pose(2,2) );
    //rot_matrix=rot_matrix.inverse ();
    tf::Quaternion q;
    rot_matrix.getRotation(q);

    nav_msgs::Odometry odom;
    geometry_msgs::Pose odom_pose;
    odom_pose.position.x=pose(0,3);
    odom_pose.position.y=pose(1,3);
    odom_pose.position.z=pose(2,3);
    odom_pose.orientation.x=q.getX();
    odom_pose.orientation.y=q.getY();
    odom_pose.orientation.z=q.getZ();
    odom_pose.orientation.w=q.getW();
    
    //set velocity to zero    
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;
    
    odom.header.stamp = ros::Time::now();    
    odom.header.frame_id = VO_FRAME;
    //odom.child_frame_id = "visual_link";
    odom.child_frame_id = DEPTH_FRAME;


    odom.pose.pose=odom_pose;
    odom_pub.publish(odom);

#ifdef PUBLISH_ODOM_PATH
    publishOdomPath(odom_pose);    
#endif
}

#ifdef PUBLISH_ODOM_PATH

void publishOdomPath(geometry_msgs::Pose &p)
{
//     nav_msgs::Path path;
    
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = VO_FRAME;
    ps.pose=p;
    odomPath.poses.push_back(ps);
    
    nav_msgs::Path newPath=odomPath;
    newPath.header.stamp = ros::Time::now();
    newPath.header.frame_id = VO_FRAME;

    odom_path_pub.publish(newPath);
}

#endif

#ifdef PUBLISH_ISAM_PATH
void publishIsamPath()
{
    std::vector<sMatrix4> vec;
    loopCl->getIsamPoses(vec);
    isamPath = odomPath;
    if(vec.size()>1)
    {
        for(int i=0;i<vec.size()-1;i++)
        {
            geometry_msgs::PoseStamped ps;
            int odomPosesIdx=odomPath.poses.size()+i-vec.size();
            ps.header=odomPath.poses[odomPosesIdx].header;
            sMatrix4 pose=vec[i];
            
            pose(0,3)-=params.volume_direction.x;
            pose(1,3)-=params.volume_direction.y;
            pose(2,3)-=params.volume_direction.z;
    //         pose=inverse(pose);
            pose=fromVisionCord(pose);
            tf::Matrix3x3 rot_matrix( pose(0,0),pose(0,1),pose(0,2),
                                pose(1,0),pose(1,1),pose(1,2),
                                pose(2,0),pose(2,1),pose(2,2) );
            
            tf::Quaternion q;
            //rot_matrix=rot_matrix.inverse();
            rot_matrix.getRotation(q);
            geometry_msgs::Pose odom_pose;
            odom_pose.position.x=pose(0,3);
            odom_pose.position.y=pose(1,3);
            odom_pose.position.z=pose(2,3);
            odom_pose.orientation.x=q.getX();
            odom_pose.orientation.y=q.getY();
            odom_pose.orientation.z=q.getZ();
            odom_pose.orientation.w=q.getW();
            
            ps.pose=odom_pose;
            isamPath.poses[odomPosesIdx]=ps;
        }
    }
            
    isamPath.header.stamp = ros::Time::now();
    isamPath.header.frame_id = VO_FRAME;
    isam_path_pub.publish(isamPath);
}
#endif

void gemLeftCallback(const std_msgs::Float32 f)
{
    if(f.data>FEET_PROB_THR)
       leftFeetValue++;
    else if(f.data<0.2)
        leftFeetValue--;
    else
        leftFeetValue=0;


    if(f.data>0.3 && f.data<0.7)
        doubleSupport++;
    else
        doubleSupport=0;
}

void gemRightCallback(const std_msgs::Float32 f)
{
    if(f.data>FEET_PROB_THR)
       rightFeetValue++;
    else if(f.data<0.2)
        rightFeetValue--;
    else
        rightFeetValue=0;

    if(f.data>0.3 && f.data<0.6)
        doubleSupport++;
    else
        doubleSupport=0;
}

void publishPoints()
{
    IcsFusion *icsFusion=loopCl->fusion();
    
    std::vector<float3> vertices;
    icsFusion->getVertices(vertices);
    
    sensor_msgs::PointCloud pcloud;
    pcloud.header.stamp = ros::Time::now();
    pcloud.header.frame_id = odom_frame;
    pcloud.points.reserve(vertices.size());
    sensor_msgs::ChannelFloat32 ch;    
    
    for(int i=0;i<vertices.size();i++)
    {
        geometry_msgs::Point32 p;
        p.x=vertices[i].x;
        p.y=vertices[i].y;
        p.z=vertices[i].z;

        pcloud.points.push_back(p);
        ch.values.push_back(1);    
    }
    pcloud.channels.push_back(ch);
    points_pub.publish(pcloud);
}

void stopBag()
{
    std_srvs::SetBool b;
    b.request.data=true;
    bagClient.call(b);
}

void contBag()
{
    std_srvs::SetBool b;
    b.request.data=false;
    bagClient.call(b);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ics_fusion_node",ros::init_options::AnonymousName);
    ros::NodeHandle n_p("~");

    std::string cam_info_topic,depth_topic,rgb_topic,gem_left_topic,gem_right_topic;
    std::string bag_name;

    if(!n_p.getParam("cam_info_topic", cam_info_topic))
    {
        cam_info_topic=std::string(CAM_INFO_TOPIC);
    }
    if(!n_p.getParam("depth_topic", depth_topic))
    {
        depth_topic=std::string(DEPTH_TOPIC);
    }
    if(!n_p.getParam("rgb_topic", rgb_topic))
    {
        rgb_topic=std::string(RGB_TOPIC);
    }
    if(!n_p.getParam("gem_left_prob", gem_left_topic))
    {
        gem_left_topic=GEM_LEFT_TOPIC;
    } 
    if(!n_p.getParam("gem_right_prob", gem_right_topic))
    {
        gem_right_topic=GEM_RIGHT_TOPIC;
    }
    n_p.getParam("bag_name", bag_name);

    n_p.param("publish_volume",publish_volume,true);
    n_p.param("publish_key_frame",publish_key_frame,true);
    n_p.param("publish_points",publish_points,true);
    n_p.param("publish_points_rate",publish_points_rate,PUBLISH_POINT_RATE);
    n_p.param("key_frame_thr",key_frame_thr,KEY_FRAME_THR);
    n_p.param("keypt_size",keypt_size,100);
    
    ROS_INFO("Depth Frame:%s",depth_frame.c_str());      
    //TODO read fusion param from yaml
    
    if(publish_volume)
        volume_pub = n_p.advertise<sensor_msgs::Image>(PUB_VOLUME_TOPIC, 1000);

    if(publish_points)
        points_pub = n_p.advertise<sensor_msgs::PointCloud>(PUB_POINTS_TOPIC, 100);

    if(publish_key_frame)
    {
        key_frame_pub = n_p.advertise<std_msgs::Float32>(PUB_KEY_FRAME_TOPIC, 100);
    }

    harris_pub = n_p.advertise<sensor_msgs::Image>(PUB_HARRIS_FRAME_TOPIC, 100);

    odom_pub = n_p.advertise<nav_msgs::Odometry>(PUB_ODOM_TOPIC, 50);

    if(publish_key_points)
    {
        pcl_pub0 = n_p.advertise<sensor_msgs::PointCloud>("point_cloud0",10);
        pcl_pub1 = n_p.advertise<sensor_msgs::PointCloud>("point_cloud1",10);
    }

#ifdef PUBLISH_ODOM_PATH
    odom_path_pub = n_p.advertise<nav_msgs::Path>(PUB_ODOM_PATH_TOPIC, 50);
#endif
    
#ifdef PUBLISH_ISAM_PATH
    isam_path_pub = n_p.advertise<nav_msgs::Path>(PUB_ISAM_PATH_TOPIC, 50);
#endif    
    //subscribe to GEM
    ros::Subscriber gem_left_sub = n_p.subscribe(gem_left_topic, 1, gemLeftCallback);
    ros::Subscriber gem_right_sub = n_p.subscribe(gem_right_topic,1, gemRightCallback);

    //smoothnetServer=new actionlib::SimpleActionClient<smoothnet_3d::SmoothNet3dAction>(SMOOTHNET_SERVER,true);
    //smoothnetServer->waitForServer();
    
    ROS_INFO("Waiting camera info");
    while(ros::ok())
    {
        sensor_msgs::CameraInfoConstPtr cam_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic);
        if(cam_info)
        {
            camInfoCallback(cam_info);
            break;
        }
    }

    std::cout<<"BAG:"<<bag_name<<std::endl;

    bagClient = n_p.serviceClient< std_srvs::SetBool>(bag_name);

    ROS_INFO("Waiting depth message");

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n_p, rgb_topic, 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n_p, depth_topic, 100);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;


    message_filters::Synchronizer<MySyncPolicy> sync( MySyncPolicy(100), rgb_sub, depth_sub);

    sync.registerCallback(boost::bind(&imageAndDepthCallback, _1, _2));
  
    contBag();
    ros::spin();
}
