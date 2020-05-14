#include <depth_rgb_sync/depth_rgb_sync_utils.h>

#define DEPTH_IN_TOPIC "/camera/depth/image_noise"
#define RGB_IN_TOPIC "/camera/rgb/image_converted"
#define RRATE 30

void depth_rgb_sync_utils::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cam0_msg = *msg;
    image_msg_new = true;
}


void depth_rgb_sync_utils::depthCb(const sensor_msgs::ImageConstPtr& msg)
{
    depth_msg = *msg;
    depth_msg_new = true;   
}


depth_rgb_sync_utils::depth_rgb_sync_utils(ros::NodeHandle nh_)
{

    nh = nh_;
    image_msg_new = false;
    depth_msg_new = false;
    seq_c = 0;
    
    int rrate;
    std::string depth_in_topic,rgb_in_topic;
    if(!nh_.getParam("depth_in_topic", depth_in_topic))
    {
        depth_in_topic = DEPTH_IN_TOPIC;
    }
    if(!nh_.getParam("rgb_in_topic", rgb_in_topic))
    {
        rgb_in_topic = RGB_IN_TOPIC;
    }
    if(!nh_.getParam("rate", rrate))
    {
        rrate = RRATE;
    }
    
    std::cout<<depth_in_topic<<std::endl;
    std::cout<<rgb_in_topic<<std::endl;
    std::cout<<rrate<<std::endl;
    
    cam0_sub = nh.subscribe(rgb_in_topic, 10,&depth_rgb_sync_utils::imageCb,this);
    depth_sub = nh.subscribe(depth_in_topic, 10,&depth_rgb_sync_utils::depthCb,this);


    

    it0 = new image_transport::ImageTransport(nh);
    image0_pub = it0->advertise("/cam0/image_raw", 10);
    it1 = new image_transport::ImageTransport(nh);
    depth_pub = it1->advertise("/depth0/image_raw", 10);

    static ros::Rate rate(rrate);

    while(ros::ok())
    {
    
        if(image_msg_new && depth_msg_new)
        {
            cam0_msg.header.stamp = ros::Time::now();
            cam0_msg.header.seq = seq_c;
            image0_pub.publish(cam0_msg);
            depth_msg.header.stamp = ros::Time::now();
            depth_msg.header.seq = seq_c;
            depth_pub.publish(depth_msg);
            image_msg_new = false;
            depth_msg_new = false;
	        seq_c++;
        }
        ros::spinOnce();
        rate.sleep();
    }
}



