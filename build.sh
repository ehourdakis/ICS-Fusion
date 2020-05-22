#!/bin/bash

CATKIN_WS="$1"
SCRIPT=$(readlink -f $0)
SCRIPT_PATH=`dirname $SCRIPT`
OLD_PATH=`pwd`

#==============
sudo apt-get install -y subversion
#==============
echo $SCRIPT_PATH $CATKIN_WS
mkdir $SCRIPT_PATH/dependencies 2>/dev/null 

cd $SCRIPT_PATH/dependencies
svn co https://svn.csail.mit.edu/isam
exit

ln -s $SCRIPT_PATH/ROS $CATKIN_WS/src/ics_fusion

source $CATKIN_WS/devel/setup.bash


cd $CATKIN_WS
catkin_make

