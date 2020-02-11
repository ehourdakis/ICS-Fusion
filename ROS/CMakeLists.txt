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
  cmake_modules
#  Eigen3
)
add_compile_options(-ggdb)
set(EIGEN3_INCLUDE_DIR "/usr/local/include/eigen3")


#find_package(catkin REQUIRED cmake_modules)
#find_package(Eigen REQUIRED)


message(WARNING EIGEN3_ROS ${EIGEN3_LIBRARIES})
message(WARNING EIGEN3_include_ROS ${EIGEN3_INCLUDE_DIRS})


catkin_package(CATKIN_DEPENDS)

include( ${CMAKE_CURRENT_LIST_DIR}/Algorithm/CMakeLists.txt)
# include(${CMAKE_CURRENT_LIST_DIR}/3DSmoothNet/CMakeLists.txt)

# include_directories(${CMAKE_CURRENT_LIST_DIR}/3DSmoothNet/src)


find_package(Eigen3   REQUIRED)
find_package(CUDA)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TOON_INCLUDE_PATHS}  
  ${CMAKE_CURRENT_LIST_DIR}/Algorithm/src
  ${CMAKE_CURRENT_LIST_DIR}/Algorithm/src/cuda_headers
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ics_fusion_node 
    ${CMAKE_CURRENT_LIST_DIR}/src/ics_fusion_node.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/src/butterworthLPF.cpp
)

#IST(APPEND G2O_LIBS
#  cxsparse
#  cholmod
#  g2o_cli g2o_ext_freeglut_minimal g2o_simulator
#  g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
#  g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
# g2o_types_sba g2o_types_slam3d g2o_csparse_extension
#  g2o_opengl_helper g2o_solver_dense g2o_stuff
#  g2o_types_sclam2d g2o_parser g2o_solver_pcg
#  g2o_types_data g2o_types_sim3
#)


target_link_libraries(ics_fusion_node
    ${catkin_LIBRARIES}  
    ics_fusion_cuda
     ${CUDA_LIBRARIES}
    isam cholmod  ${OpenCV_LIBS}  ${Eigen3_LIBRARIES}
    ${G2O_LIBS}
)