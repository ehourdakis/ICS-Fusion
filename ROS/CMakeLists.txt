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

#add_compile_options(-ggdb)
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

