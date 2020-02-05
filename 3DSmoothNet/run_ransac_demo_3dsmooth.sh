#!/bin/bash

ENV="./env"
cd "/home/tavu/workspace/ICS-Fusion/3DSmoothNet"
# cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 
source ${ENV}/bin/activate


frame1="$1"
frame2="$2"

point_cloud_file1="./data/ply/f_${frame1}_vertices.ply"
point_cloud_file2="./data/ply/f_${frame2}_vertices.ply"

descr_file1="./data/descr/frame${frame1}.csv"
descr_file2="./data/descr/frame${frame2}.csv"

keypoints_file1="./data/keypts/f_${frame1}_keypoints.txt"
keypoints_file2="./data/keypts/f_${frame2}_keypoints.txt"

output_file="./data/transformations/frame${frame2}.txt"

python ransac_demo.py \
    $point_cloud_file1 \
    $point_cloud_file2 \
    $descr_file1 \
    $descr_file2 \
    $keypoints_file1 \
    $keypoints_file2 \
    $output_file