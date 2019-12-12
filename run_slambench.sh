#!/bin/bash

DATASET="datasets/ICL_NUIM/living_room_traj2_loop.slam"
#DATASET="datasets/TUM/freiburg1/rgbd_dataset_freiburg1_360.slam"
#DATASET="datasets/TUM/freiburg1/rgbd_dataset_freiburg1_xyz.slam"


LIBRARY="libicsFusion-cuda-library.so"
BENCHMARK="benchmark_loader"
DEMO="pangolin_loader"

COMMAND=$BENCHMARK
./build/bin/$COMMAND -i $DATASET -load ./build/lib/$LIBRARY  
