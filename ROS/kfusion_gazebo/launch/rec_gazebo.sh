#!/bin/bash

rosbag record -O "$1" \
    /depth0/image_raw \
    /cam0/image_raw \
    /joint_states \
    /tf \
    /tf_static \
    /odom
    