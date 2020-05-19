#!/bin/sh
roslaunch ics_fusion ics_fusion_gazebo.launch  bag_name:=`rosservice list|grep pause_playback`
