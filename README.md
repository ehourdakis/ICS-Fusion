## icsfusion

A SLAM system

## For SLAMBENCH2 usage:
go to slambench bencharks direcory and link Slambench2_benchmark there with  name `icsFusion`.

```console
cd ${SLAMBENCH}/slambench2/benchmarks/
ln -s ${icsFusion}/Slambench2_benchmark icsFusion
```

## compile

make slambench APPS=icsFusion

## For ROS usage

go to catkin source directory and link ROS directory there


```console
cd ${catkin_ws}/src
ln -s ${icsFusion}/ROS .
```
## compile

catkin_make

## Smoothnet3d needs 3.7GB on GPU
## CUDA 9.0 PCL 1.7 CUDNN 7.0.5
# Get CUDNN 7.0.5 for CUDA 9.0
https://developer.nvidia.com/rdp/cudnn-archive
#follow the instructions 
https://docs.nvidia.com/deeplearning/sdk/cudnn-install/index.html#installlinux-tar

# install tensorflow-gpu 1.10
pip install tensorflow-gpu==1.10
