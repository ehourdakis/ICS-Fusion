cmake_minimum_required(VERSION 3.3)
project(smoothnet_3d_lrf)

add_compile_options(-std=c++11 -fopenmp)

set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.7")
set(EIGEN_INCLUDE_DIRS "/usr/local/include/eigen3")

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/src/cpp
    ${PCL_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    /usr/include/python3.5m
)

set(cpp_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/cpp/smoothnetinterf.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/cpp/smoothnetcore.cpp
)

add_library(smoothnet_3d_lrf 
    SHARED
    ${cpp_srcs}
)

# -fopenmp

target_link_libraries(smoothnet_3d_lrf 
    pcl_common    
    pcl_kdtree
    pcl_filters
)

SET_TARGET_PROPERTIES(smoothnet_3d_lrf PROPERTIES COMPILE_FLAGS "-fPIC" )
