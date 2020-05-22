#!/bin/bash

CATKIN_WS="$1"
SCRIPT=$(readlink -f $0)
SCRIPT_PATH=`dirname $SCRIPT`
OLD_PATH=`pwd`

if [ -z "$CATKIN_WS" ]
then
      echo "Please specify the catking workspace path."
      echo "Usage:"
      echo "./build.sh <catking_workspace_path>"
      exit -1
fi


#==============Install depentencies through apt===============
sudo apt-get install -y subversion libsuitesparse-dev 
#=============================================================
echo $SCRIPT_PATH $CATKIN_WS
mkdir -p $SCRIPT_PATH/dependencies 
cd $SCRIPT_PATH/dependencies


#=========================EIGEN3==========================
cd $SCRIPT_PATH/dependencies
git clone https://gitlab.com/libeigen/eigen.git
#=======================================================


#=========================ISAM==========================
git clone https://github.com/ori-drs/isam.git
cd isam
git apply $SCRIPT_PATH/isam_pic.patch
mkdir -p build 
cd build
cmake ..
make
#=======================================================

#=========================TOON==========================
cd $SCRIPT_PATH/dependencies
git clone https://github.com/edrosten/TooN.git
cd $SCRIPT_PATH/dependencies/TooN
./configure && make
#=======================================================

if [ -e "$CATKIN_WS/src/ics_fusion" ]; then
  echo "[WARNING] Directory 'ics_fusion' alredy exist into $CATKIN_WS/src"
else
    ln -s $SCRIPT_PATH/ROS/ics_fusion $CATKIN_WS/src/
fi


source $CATKIN_WS/devel/setup.bash


cd $CATKIN_WS
catkin_make
rospack profile
