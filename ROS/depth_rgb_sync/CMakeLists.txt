cmake_minimum_required(VERSION 2.8.3)
project(depth_rgb_sync)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  message_filters
)

catkin_package(CATKIN_DEPENDS)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(depth_rgb_sync_node src/depth_rgb_sync_node.cpp src/depth_rgb_sync_utils.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(depth_rgb_sync_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(depth_rgb_sync_node ${stereo_imu_sync_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 ## Specify libraries to link a library or executable target against
target_link_libraries(depth_rgb_sync_node
    ${catkin_LIBRARIES}
)
