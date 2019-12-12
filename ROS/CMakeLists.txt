cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 OLD) #STOP stupid warnings
project(ite_fusion)

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

# find_package(TooN   REQUIRED)
# find_package(Eigen3   REQUIRED)

set(TOON_INCLUDE_PATHS "/home/tavu/src/TooN")
set(EIGEN3_INCLUDE_DIR "/home/tavu/src/eigen")
# 
# find_package(CUDA)
# SET( CUDA_PROPAGATE_HOST_FLAGS OFF ) 
# set(kfusion_cuda_srcs src/kernels.cu)
# 
# cuda_add_library(ite_fusion_cuda    SHARED  ${kfusion_cuda_srcs}  OPTIONS -O3 -std=c++11 -Xcompiler -fPIC -D_MWAITXINTRIN_H_INCLUDED -D_FORCE_INLINES 
#     -I${CMAKE_CURRENT_SOURCE_DIR}/src/cuda_headers 
#     -I${CMAKE_CURRENT_SOURCE_DIR}/src 
#     -I${TOON_INCLUDE_PATHS}
#     -I${EIGEN3_INCLUDE_DIR}
#     )
message(WARNING ${TOON_INCLUDE_PATHS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TOON_INCLUDE_PATHS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/Algorithm/src
  ${CMAKE_CURRENT_LIST_DIR}/Algorithm/src/cuda_headers  
)

add_executable(ite_fusion_node 
    ${CMAKE_CURRENT_LIST_DIR}/src/ite_fusion_node.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/src/image_process.cpp
)

target_link_libraries(ite_fusion_node
    ${catkin_LIBRARIES}  
    ite_fusion_cuda
)



