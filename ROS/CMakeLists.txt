cmake_minimum_required(VERSION 3.3)
project(ics_fusion)

find_package(CUDA)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs
    image_transport
    tf2
    tf2_ros
)

# 
# LIST(APPEND G2O_LIBS
#   cxsparse
#   cholmod
#   g2o_cli g2o_ext_freeglut_minimal g2o_simulator
#   g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
#   g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
#   g2o_types_sba g2o_types_slam3d g2o_csparse_extension
#   g2o_opengl_helper g2o_solver_dense g2o_stuff
#   g2o_types_sclam2d g2o_parser g2o_solver_pcg
#   g2o_types_data g2o_types_sim3
# )



catkin_package(CATKIN_DEPENDS 
    message_runtime 
    roscpp rospy std_msgs    
    DEPENDS
    eigen3
)

include( ${CMAKE_CURRENT_LIST_DIR}/Algorithm/CMakeLists.txt)

#SET LD_LIBRARY_PATH ME TO PCL



include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include 
    ${CMAKE_CURRENT_LIST_DIR}/Algorithm/src
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}/src    
)

add_executable(ics_fusion_node 
    src/ics_fusion_node.cpp
)

add_compile_options(-ggdb)
# SET_TARGET_PROPERTIES(talker PROPERTIES COMPILE_FLAGS "-ggdb" )

message(WARNING PCL: ${PCL_LIBRARIES})

target_link_libraries(ics_fusion_node 
    ics_fusion_lib
    ${catkin_LIBRARIES}
    ${PCL_LIBARIES}    
    pcl_common    
    pcl_kdtree
    pcl_filters
)

