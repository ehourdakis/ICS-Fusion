#include <ros/ros.h>

#include <std_msgs/String.h>
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

#include "image_process.h"

#include <kparams.h>

#define CAM_INFO_TOPIC "/camera/depth/camera_info"
#define RGB_TOPIC "/camera/rgb/image_rect_color"
#define DEPTH_TOPIC "/camera/depth/image_rect"

#define PUB_VOLUME_TOPIC "/itefusion/volume_rendered"
#define PUB_ODOM_TOPIC "/itefusion/odom"
#define PUB_POINTS_TOPIC "/itefusion/pointCloud"
#define PUB_IMAGE_TOPIC "/itefusion/volume_rgb"

#define DEPTH_FRAME "camera_rgb_optical_frame"
#define VO_FRAME "visual_odom"
#define ODOM_FRAME "odom"
#define BASE_LINK "base_link"

#define PUBLISH_POINT_RATE 10
#define PUBLISH_IMAGE_RATE 1


typedef unsigned char uchar;

kparams_t params;

//Buffers
uint16_t *inputDepth=0;
//uint8_t *inputRgb=0; //rgb8
float *inputDepthFl=0;
uchar3 *inputRGB;
uchar4 *depthRender;
uchar4 *trackRender;
uchar3 *volumeRender;

//IteFusion
Kfusion *iteFusion=nullptr;
//Image_process
ImageProcess *imageProcess;

std::string odom_in_topic;

int frame = 0;
int odom_delay;
int odom_rec=0;
int odom_counter = 0;

//other params
bool publish_volume=true;
bool publish_points=true;
int publish_points_rate;
bool publish_image=false;
int publish_image_rate;


//ros pub/subs
ros::Subscriber cam_info_sub;
ros::Publisher volume_pub;
ros::Publisher odom_pub ;
ros::Publisher points_pub;
ros::Publisher image_pub;
nav_msgs::Odometry odom_;

//hold previous pose
Matrix4 pose_old;
bool hasPoseOld=false;

//frames
std::string depth_frame,vo_frame,base_link_frame,odom_frame;

//Transformations
geometry_msgs::TransformStamped odom_to_vo,vo_to_odom;
geometry_msgs::TransformStamped cam_to_base,base_to_cam;
tf2_ros::Buffer tfBuffer;

//functions
void initIteFusion();
void publishVolume();
void publishImage();
void publishOdom();
void publishPoints();
void depthCallback(const sensor_msgs::ImageConstPtr &depth);
geometry_msgs::Pose transform2pose(const geometry_msgs::Transform &trans);


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

    params=par;
}

void odomCallback(nav_msgs::OdometryConstPtr odom)
{
    ROS_INFO("odom %d",odom_rec);
    if(odom_rec!=odom_delay)
    {
        odom_rec++; 
        return;
    }
    odom_rec++; 


    geometry_msgs::Pose odomP;
    geometry_msgs::Pose poseTmp;

    /*Create tf transformation from odom message*/
    geometry_msgs::TransformStamped base_to_odom;
    base_to_odom.header.frame_id=odom_frame;
    base_to_odom.child_frame_id=base_link_frame;
    
    base_to_odom.transform.translation.x=odom->pose.pose.position.x;
    base_to_odom.transform.translation.y=odom->pose.pose.position.y;
    base_to_odom.transform.translation.z=odom->pose.pose.position.z;
    
    base_to_odom.transform.rotation.x=odom->pose.pose.orientation.x;
    base_to_odom.transform.rotation.y=odom->pose.pose.orientation.y;
    base_to_odom.transform.rotation.z=odom->pose.pose.orientation.z;
    base_to_odom.transform.rotation.w=odom->pose.pose.orientation.w;
    

    odomP=transform2pose(cam_to_base.transform);
    try
    {
        tf2::doTransform(odomP,odomP, base_to_odom);
        tf2::doTransform(odomP,odomP, odom_to_vo);
    }
    catch (tf2::TransformException &ex) 
    {
        
        ROS_WARN("Odom transformation failure %s\n", ex.what()); //Print exception which was caught    
        return;
    }

    Matrix4 pose, delta_pose;

    //New Odometry
    tf::Quaternion q(odomP.orientation.x,
                     odomP.orientation.y,
                     odomP.orientation.z,
                     odomP.orientation.w);

    
   tf::Matrix3x3 rot_matrix(q);

    for(int i=0;i<3;i++)
    {
        tf::Vector3 vec=rot_matrix.getRow(i);
        pose.data[i].x=vec.getX();
        pose.data[i].y=vec.getY();
        pose.data[i].z=vec.getZ();
        pose.data[i].w=0;
    }
    pose.data[3].x=0;
    pose.data[3].y=0;
    pose.data[3].z=0;
    pose.data[3].w=1;

    
    pose.data[0].w=odomP.position.x;
    pose.data[1].w=odomP.position.y; 
    pose.data[2].w=odomP.position.z;
        
    pose.data[0].w += params.volume_direction.x;
    pose.data[1].w += params.volume_direction.y;
    pose.data[2].w += params.volume_direction.z;
    
    
    if(!hasPoseOld)
    {
        pose_old=pose;
        hasPoseOld=true;
        return;
    }
    if(iteFusion==nullptr)
        return;    
    
    delta_pose = inverse(pose_old) * pose;
    pose_old=pose;
    hasPoseOld=true;

    Matrix4 p=iteFusion->getPose();
    Matrix4 p_new = iteFusion->getPose() * delta_pose;

    iteFusion->setPose(p_new);

    //Integrate
    if(!iteFusion->integration(frame))
        ROS_ERROR("integration faild");

    //Raycasting
    iteFusion->raycasting(frame);

    if(publish_volume)
    {
        iteFusion->renderVolume(volumeRender);
        publishVolume();
    }
    
//     imageProcess->setImages(inputRGB,volumeRender);

    if(publish_points && frame % publish_points_rate ==0)
         publishPoints();

//     iteFusion->renderImage(volumeRender,
//                         params.computationSize,
//                         params.camera,
//                         0.75 * params.mu);
// 
// 
//     if(publish_image && frame % publish_image_rate ==0)
//          publishImage();

    frame++;
}

void imageCallback(const sensor_msgs::ImageConstPtr &rgb,const sensor_msgs::ImageConstPtr &depth)
{
//    ROS_INFO("EDO");

    if(strcmp(rgb->encoding.c_str(), "rgb8")==0) //rgb8
    {
        memcpy(inputRGB,rgb->data.data(),params.inputSize.y*params.inputSize.x*sizeof(uchar)*3 );
        depthCallback(depth);
    }
    else
    {
        ROS_ERROR("Not supported rgb format.");
        return;
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr &depth)
{
    if(iteFusion==nullptr)
    {
        params.inputSize.y=depth->height;
        params.inputSize.x=depth->width;
        initIteFusion();
    }

    const float *in_data=(const float*)depth->data.data();
    if(strcmp(depth->encoding.c_str(), "32FC1")==0) //32FC1
    {
        if(inputDepthFl==0)
            inputDepthFl=new float[params.inputSize.x * params.inputSize.y];
            
        memcpy(inputDepthFl,depth->data.data(),params.inputSize.y*params.inputSize.x*sizeof(float) );
        iteFusion->preprocessing2(inputDepthFl,inputRGB, params.inputSize);
    }
    else if(strcmp(depth->encoding.c_str(), "16UC1")==0) //16UC1
    {
        if(inputDepth==0)
            inputDepth = new uint16_t[params.inputSize.x * params.inputSize.y];   
        
        memcpy(inputDepth,depth->data.data(),params.inputSize.y*params.inputSize.x*2);
        iteFusion->preprocessing(inputDepth,inputRGB, params.inputSize);
    }
    else
    {
        ROS_ERROR("Not supported depth format.");
        return;
    }

    bool track_success=iteFusion->tracking(frame);
    odom_rec=0;
    if(! track_success)
       ROS_ERROR("Tracking faild!");

    
    
        //Integrate
    if(!iteFusion->integration(frame))
        ROS_ERROR("integration faild");

    //Raycasting
    iteFusion->raycasting(frame);

    if(publish_volume)
    {
        iteFusion->renderVolume(volumeRender);
        publishVolume();
    }
    
    publishOdom();
//     imageProcess->setImages(inputRGB,volumeRender);

    if(publish_points && frame % publish_points_rate ==0)
         publishPoints();

    frame++;
}

void camInfoCallback(sensor_msgs::CameraInfoConstPtr msg)
{
    params.camera =  make_float4(msg->K[0],msg->K[4],msg->K[2],msg->K[5]);
}

void initIteFusion()
{
    params.computationSize = make_uint2(
                params.inputSize.x / params.compute_size_ratio,
                params.inputSize.y / params.compute_size_ratio);

    
    ROS_INFO("camera is = %f, %f, %f, %f",
             params.camera.x,
             params.camera.y,
             params.camera.z,
             params.camera.w);
    
    Matrix4 poseMatrix;    
    
    for(int i=0;i<4;i++)
    {
        poseMatrix.data[i].x = 0;
        poseMatrix.data[i].y = 0;
        poseMatrix.data[i].z = 0;        
    }
    
     poseMatrix.data[0].x = 1;
     poseMatrix.data[1].y = 1;
     poseMatrix.data[2].z = 1;
    
    poseMatrix.data[0].w =  0;
    poseMatrix.data[1].w =  0;
    poseMatrix.data[2].w =  0;
    poseMatrix.data[3].w =  1;
    
    poseMatrix.data[0].w +=  params.volume_direction.x;
    poseMatrix.data[1].w +=  params.volume_direction.y;
    poseMatrix.data[2].w +=  params.volume_direction.z;

    inputRGB     = new uchar3[params.inputSize.x * params.inputSize.y];
    depthRender  = new uchar4[params.computationSize.x * params.computationSize.y];
    volumeRender = new uchar3[params.computationSize.x * params.computationSize.y];
    
    iteFusion = new Kfusion(params,poseMatrix);

    imageProcess = new ImageProcess(params.computationSize);

}

void publishImage()
{
    sensor_msgs::Image image;
    image.header.stamp=ros::Time::now();

    image.height=params.inputSize.y;
    image.width=params.inputSize.x;

    int step_size=sizeof(uchar)*3;
    image.is_bigendian=0;
    image.step=step_size*image.width;
    image.header.frame_id=std::string("IteFusion_volume");
//    image.encoding=std::string("bgra8");
    image.encoding=std::string("rgb8");

    uchar *ptr=(uchar*)volumeRender;
    image.data=std::vector<uchar>(ptr ,ptr+(params.computationSize.x * params.computationSize.y*step_size) );
    image_pub.publish(image);
}

void publishVolume()
{
    sensor_msgs::Image image;
    image.header.stamp=ros::Time::now();
    
    image.height=params.inputSize.y;
    image.width=params.inputSize.x;
    
    int step_size=sizeof(uchar)*4;
    image.is_bigendian=0;
    image.step=step_size*image.width;
    image.header.frame_id=std::string("IteFusion_volume");
    image.encoding=std::string("bgra8");

    uchar *ptr=(uchar*)volumeRender;
    image.data=std::vector<uchar>(ptr ,ptr+(params.computationSize.x * params.computationSize.y*step_size) );
    volume_pub.publish(image);
}

void publishOdom()
{
    Matrix4 pose = iteFusion->getPose();
    tf::Vector3 vec[3];
    for(int i=0;i<3;i++)
    {
        vec[i]=tf::Vector3(pose.data[i].x,pose.data[i].y,pose.data[i].z);
    }

    tf::Matrix3x3 rot_matrix(vec[0].getX(),vec[0].getY(),vec[0].getZ(),
                             vec[1].getX(),vec[1].getY(),vec[1].getZ(),
                             vec[2].getX(),vec[2].getY(),vec[2].getZ() );

    tf::Quaternion q;
    rot_matrix.getRotation(q);

    nav_msgs::Odometry odom;
    geometry_msgs::Pose odom_pose;
    geometry_msgs::TransformStamped cam_to_vo;
    cam_to_vo.header.frame_id=depth_frame;
    cam_to_vo.child_frame_id=base_link_frame;

    odom.header.stamp = ros::Time::now();

    cam_to_vo.transform.translation.x = pose.data[0].w-params.volume_direction.x;
    cam_to_vo.transform.translation.y = pose.data[1].w-params.volume_direction.y;
    cam_to_vo.transform.translation.z = pose.data[2].w-params.volume_direction.z;

    cam_to_vo.transform.rotation.x=q.getX();
    cam_to_vo.transform.rotation.y=q.getY();
    cam_to_vo.transform.rotation.z=q.getZ();
    cam_to_vo.transform.rotation.w=q.getW();

    //set velocity to zero    
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;
    
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = "visual_link";

    odom_pose=transform2pose(base_to_cam.transform);
    try
    {
        tf2::doTransform(odom_pose,odom_pose, cam_to_vo);
        tf2::doTransform(odom_pose,odom_pose, vo_to_odom);
    }
    catch (tf2::TransformException &ex)
    {

        ROS_WARN("Odom transformation failure %s\n", ex.what()); //Print exception which was caught
        return;
    }

    odom.pose.pose=odom_pose;
    odom_pub.publish(odom);
}

void publishPoints()
{
    std::vector<float3> vertices;
    iteFusion->getVertices(vertices);
    
    sensor_msgs::PointCloud pcloud;
    pcloud.header.stamp = ros::Time::now();
    pcloud.header.frame_id = odom_frame;
    pcloud.points.reserve(vertices.size());
    sensor_msgs::ChannelFloat32 ch;    
    
    for(int i=0;i<vertices.size();i++)
    {
        float3 vertex=vertices[i];
        geometry_msgs::Point point;

        point.x= vertex.x -params.volume_direction.x;
        point.y= vertex.y -params.volume_direction.y;
        point.z= vertex.z -params.volume_direction.z;

        try
        {
            tf2::doTransform(point,point, vo_to_odom);
        }
        catch (tf2::TransformException &ex)
        {

            ROS_WARN("Odom transformation failure %s\n", ex.what()); //Print exception which was caught
            return;
        }

        geometry_msgs::Point32 p;
        p.x=point.x;
        p.y=point.y;
        p.z=point.z;

        pcloud.points.push_back(p);
        ch.values.push_back(1);    
    }
    pcloud.channels.push_back(ch);
    points_pub.publish(pcloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ite_fusion_node",ros::init_options::AnonymousName);
    ros::NodeHandle n_p("~");

    std::string cam_info_topic,depth_topic,rgb_topic;
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
    if(!n_p.getParam("odom_input_topic", odom_in_topic))
    {
        odom_in_topic=std::string(PUB_ODOM_TOPIC);
    }    
    if(!n_p.getParam("odom_delay", odom_delay))
    {
        odom_delay=3;
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
    if(!n_p.getParam("publish_image", publish_image))
    {
        publish_image=true;
    }
    if(!n_p.getParam("publish_image_rate", publish_image_rate))
    {
        publish_image_rate=PUBLISH_IMAGE_RATE;
    }
    if(!n_p.getParam("vo_frame", vo_frame))
    {
        vo_frame=VO_FRAME;
    }    
    if(!n_p.getParam("depth_frame", depth_frame))
    {
        depth_frame=DEPTH_FRAME;
    }
    if(!n_p.getParam("base_link_frame", base_link_frame))
    {
        base_link_frame=BASE_LINK;
    }
    if(!n_p.getParam("odom_frame", odom_frame))
    {
        odom_frame=ODOM_FRAME;
    }

    ROS_INFO("Depth Frame:%s",depth_frame.c_str());
           
    readIteFusionParams(n_p);
    
    if(publish_volume)
        volume_pub = n_p.advertise<sensor_msgs::Image>(PUB_VOLUME_TOPIC, 1000);

    if(publish_points)
        points_pub = n_p.advertise<sensor_msgs::PointCloud>(PUB_POINTS_TOPIC, 100);

    if(publish_image)
        image_pub = n_p.advertise<sensor_msgs::Image>(PUB_IMAGE_TOPIC, 1000);

    odom_pub = n_p.advertise<nav_msgs::Odometry>(PUB_ODOM_TOPIC, 50);

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

    ROS_INFO("Waiting tf transformation");
// #if 0
    do
    {
        tf2_ros::TransformListener tfListener(tfBuffer);        
        while(true)
        {
            try
            {
//                 odom_to_vo = tfBuffer.lookupTransform(depth_frame,odom_frame, ros::Time(0));
                odom_to_vo = tfBuffer.lookupTransform(depth_frame,"kinect", ros::Time(0));
                cam_to_base = tfBuffer.lookupTransform(base_link_frame,depth_frame, ros::Time(0));
                break;
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught    
                ros::Duration(0.2).sleep();
            }
        }        
        
        tf2::Transform tr;
        tf2::fromMsg(odom_to_vo.transform,tr);
        tr=tr.inverse();
        vo_to_odom.transform=tf2::toMsg(tr);

        tf2::fromMsg(cam_to_base.transform,tr);
        tr=tr.inverse();
        base_to_cam.transform=tf2::toMsg(tr);
    }while(false);
// #endif
//     ros::Subscriber odom_sub = n_p.subscribe(odom_in_topic, odom_delay+1, odomCallback);
    ROS_INFO("Waiting depth message");

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n_p, rgb_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n_p, depth_topic, 1);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

//    image_transport::ImageTransport it(n_p);
//    image_transport::Subscriber sub = it.subscribe(depth_topic, 1, depthCallback);
    
    ros::spin();
}
