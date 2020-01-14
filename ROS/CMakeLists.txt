cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 OLD) #STOP stupid warnings
project(ics_fusion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  tf2
  tf2_ros
)

catkin_package(CATKIN_DEPENDS)

include( ${CMAKE_CURRENT_LIST_DIR}/Algorithm/CMakeLists.txt)
find_package(Eigen3   REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TOON_INCLUDE_PATHS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/Algorithm/src
  ${CMAKE_CURRENT_LIST_DIR}/Algorithm/src/cuda_headers  
)

add_executable(ics_fusion_node 
    ${CMAKE_CURRENT_LIST_DIR}/src/ics_fusion_node.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/src/image_process.cpp
)

target_link_libraries(ics_fusion_node
    ${catkin_LIBRARIES}  
    ics_fusion_cuda
)



