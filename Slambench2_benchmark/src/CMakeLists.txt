cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations ") # Eigen raises deprecated declarations warning.

include(${CMAKE_CURRENT_LIST_DIR}/../Algorithm/CMakeLists.txt)

include_directories(
    ${SLAMBENCH_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../Algorithm/src
)

add_library(${appname}-cuda  icsFusion_library.cpp )
SET_TARGET_PROPERTIES(${appname}-cuda PROPERTIES COMPILE_FLAGS "-fPIC")


target_link_libraries(${appname}-cuda      
    ics_fusion_lib
    ${common_libraries}
    pcl_common    
    pcl_kdtree
    pcl_filters
)

install(TARGETS ${appname}-cuda  DESTINATION  lib/)

generate_slam_library(${appname}-cuda "")
