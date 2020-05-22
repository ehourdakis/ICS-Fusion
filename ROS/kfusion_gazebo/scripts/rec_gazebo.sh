#!/bin/bash

rosbag record -O "$1" \
    /depth0/image_raw \
    /cam0/image_raw \
    /camera/depth/camera_info \
    /camera/rgb/camera_info \
    /camera/depth/image_raw \
    /camera/rgb/image_raw \
    /joint_states \
    /tf \
    /tf_static \
    /odom \
    /cmd_vel_mux/input/teleop \
    /clock \
    /ground_truth/base