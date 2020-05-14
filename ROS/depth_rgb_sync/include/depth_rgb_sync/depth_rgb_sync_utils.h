#ifndef DEPTH_RGB_SUNC_UTILS_H
#define DEPTH_RGB_SUNC_UTILS_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

class depth_rgb_sync_utils
{
    public:
        depth_rgb_sync_utils(ros::NodeHandle nh_);
    private:
        ros::NodeHandle nh;
        bool image_msg_new,depth_msg_new;

        sensor_msgs::Image cam0_msg, depth_msg;
        int  seq_c;
        ros::Subscriber cam0_sub, depth_sub;
        image_transport::ImageTransport* it0, *it1;
        image_transport::Publisher image0_pub;
        image_transport::Publisher depth_pub;

        void imageCb(const sensor_msgs::ImageConstPtr& msg);
        void depthCb(const sensor_msgs::ImageConstPtr& msg);
};

#endif //DEPTH_RGB_SUNC_UTILS_H
