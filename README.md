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

## Reference

```
http://3dmatch.cs.princeton.edu/
```