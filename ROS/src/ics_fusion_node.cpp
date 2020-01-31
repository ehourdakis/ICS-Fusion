#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
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

#include<closeloop.h>
#include<kparams.h>
#include<icsFusion.h>
#include<volume.h>

#include"butterworthLPF.h"

#define CAM_INFO_TOPIC "/camera/depth/camera_info"
#define RGB_TOPIC "/camera/rgb/image_rect_color"
#define DEPTH_TOPIC "/camera/depth/image_rect"
#define GEM_LEFT_TOPIC "/gem/LLegContactProbability"
#define GEM_RIGHT_TOPIC "/gem/RLegContactProbability"


#define PUB_VOLUME_TOPIC "/ics_fusion/volume_rendered"
#define PUB_ODOM_TOPIC "/ics_fusion/odom"
#define PUB_POINTS_TOPIC "/ics_fusion/pointCloud"
#define PUB_IMAGE_TOPIC "/ics_fusion/volume_rgb"
#define PUB_LEFT_PROB_TOPIC "/ics_fusion/left_foot_prob"
#define PUB_RIGHT_PROB_TOPIC "/ics_fusion/right_foot_prob"


#define DEPTH_FRAME "camera_rgb_optical_frame"
#define VO_FRAME "visual_odom"
#define ODOM_FRAME "odom"
#define BASE_LINK "base_link"

#define DEFAULT_CUT_OFF 0.7

#define PUBLISH_POINT_RATE 10
#define PUBLISH_IMAGE_RATE 1

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
bool publish_foot_prob=true;
int publish_points_rate;

//ros publishers
ros::Publisher volume_pub;
ros::Publisher odom_pub ;
ros::Publisher points_pub;
ros::Publisher left_prob_pub;
ros::Publisher right_prob_pub;


//frames
std::string depth_frame,vo_frame,base_link_frame,odom_frame;

//functions
void publishVolumeProjection();
void publishOdom();
void publishPoints();

//Low pass filters for GEM
butterworthLPF leftFilter;
butterworthLPF rightFilter;
double leftFeetValue=0;
double rightFeetValue=0;

geometry_msgs::Pose transform2pose(const geometry_msgs::Transform &trans);
void publishFeetProb();

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

void imageAndDepthCallback(const sensor_msgs::ImageConstPtr &rgb,const sensor_msgs::ImageConstPtr &depth)
{
//    ROS_INFO("EDO");

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
    
    publishOdom();
    
    if(publish_volume)
    {
        publishVolumeProjection();
    }
    
    
    if(publish_points && frame % publish_points_rate ==0)
         publishPoints();
    
    if(publish_foot_prob)
    {
        publishFeetProb();
    }
    
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
    
    Matrix4 poseMatrix;  
    poseMatrix(0,3)=params.volume_direction.x;
    poseMatrix(1,3)=params.volume_direction.y;
    poseMatrix(2,3)=params.volume_direction.z;
  

    inputRGB     = new uchar3[params.inputSize.x * params.inputSize.y];    
    volumeRender = new uchar3[params.computationSize.x * params.computationSize.y];
    
    loopCl=new CloseLoop(params,poseMatrix);
}

void publishVolumeProjection()
{
    IcsFusion *icsFusion=loopCl->fusion();
    icsFusion->renderImage(volumeRender);
    
    sensor_msgs::Image image;
    image.header.stamp=ros::Time::now();
    
    image.height=params.inputSize.y;
    image.width=params.inputSize.x;
    
    int step_size=sizeof(uchar)*3;
    image.is_bigendian=0;
    image.step=step_size*image.width;
    image.header.frame_id=std::string("IcsFusion_volume");
    image.encoding=std::string("rgb8");

    uchar *ptr=(uchar*)volumeRender;
    image.data=std::vector<uchar>(ptr ,ptr+(params.computationSize.x * params.computationSize.y*step_size) );
    volume_pub.publish(image);
}

void publishOdom()
{
    sMatrix4 pose = loopCl->getPose();

    /*
    tf::Vector3 vec[3];
    for(int i=0;i<3;i++)
    {
        vec[i]=tf::Vector3(pose.data[i].x,pose.data[i].y,pose.data[i].z);
    }
    
    tf::Matrix3x3 rot_matrix(vec[0].getX(),vec[0].getY(),vec[0].getZ(),
                             vec[1].getX(),vec[1].getY(),vec[1].getZ(),
                             vec[2].getX(),vec[2].getY(),vec[2].getZ() );
    */
    tf::Matrix3x3 rot_matrix( pose(0,0),pose(0,1),pose(0,2),
                              pose(1,0),pose(1,1),pose(1,2),
                              pose(2,0),pose(2,1),pose(2,2) );
    
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
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = "visual_link";


    odom.pose.pose=odom_pose;
    odom_pub.publish(odom);
}

void gemLeftCallback(const std_msgs::Float32 f)
{
    ROS_INFO("Left foot probability:%f\n",f.data);
    leftFeetValue=leftFilter.filter(f.data);
}

void gemRightCallback(const std_msgs::Float32 f)
{
    ROS_INFO("Right foot probability:%f\n",f.data);
    rightFeetValue=rightFilter.filter(f.data);
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

void publishFeetProb()
{
    std_msgs::Float32 leftFoot;
    leftFoot.data=leftFeetValue;
    
    std_msgs::Float32 rightFoot;
    leftFoot.data=rightFeetValue;
    
    left_prob_pub.publish(leftFoot);
    right_prob_pub.publish(rightFoot);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ite_fusion_node",ros::init_options::AnonymousName);
    ros::NodeHandle n_p("~");

    std::string cam_info_topic,depth_topic,rgb_topic,gem_left_topic,gem_right_topic;

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
    if(!n_p.getParam("publish_volume", publish_volume))
    {
        publish_volume=true;
    }
    if(!n_p.getParam("publish_points", publish_points))
    {
        publish_points=true;
    } 
    if(!n_p.getParam("publish_points_rate", publish_points_rate))
    {
        publish_points_rate=PUBLISH_POINT_RATE;
    }
    if(!n_p.getParam("gem_left_prob", gem_left_topic))
    {
        gem_left_topic=GEM_LEFT_TOPIC;
    } 
    if(!n_p.getParam("gem_right_prob", gem_right_topic))
    {
        gem_right_topic=GEM_RIGHT_TOPIC;
    }
    
    double cut_off;
    n_p.param("cut_off",cut_off,DEFAULT_CUT_OFF);
    n_p.param("publish_foot_prob",publish_foot_prob,true);
    
    
    ROS_INFO("Depth Frame:%s",depth_frame.c_str());
    ROS_INFO("Gem cut off:%f",cut_off);
      
    //TODO read fusion param from yaml
    //readIteFusionParams(n_p);
    
    if(publish_volume)
        volume_pub = n_p.advertise<sensor_msgs::Image>(PUB_VOLUME_TOPIC, 1000);

    if(publish_points)
        points_pub = n_p.advertise<sensor_msgs::PointCloud>(PUB_POINTS_TOPIC, 100);

    if(publish_foot_prob)
    {
        left_prob_pub = n_p.advertise<std_msgs::Float32>(PUB_LEFT_PROB_TOPIC, 100);
        right_prob_pub = n_p.advertise<std_msgs::Float32>(PUB_RIGHT_PROB_TOPIC, 100);
    }

    odom_pub = n_p.advertise<nav_msgs::Odometry>(PUB_ODOM_TOPIC, 50);
    
    leftFilter.init("left-leg",100,cut_off);
    rightFilter.init("right-leg",100,cut_off);
    
    //subscribe to GEM
    ros::Subscriber gem_left_sub = n_p.subscribe(gem_left_topic, 1000, gemLeftCallback);
    ros::Subscriber gem_right_sub = n_p.subscribe(gem_right_topic,1000, gemRightCallback);

    
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

  
    ROS_INFO("Waiting depth message");

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n_p, rgb_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n_p, depth_topic, 1);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&imageAndDepthCallback, _1, _2));
  
    ros::spin();
}
