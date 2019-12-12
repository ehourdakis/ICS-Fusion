cmake_minimum_required(VERSION 2.8.3)
# find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations ") # Eigen raises deprecated declarations warning.
set(MRPT_BASE "/usr/include/mrpt/base/include")
set(MRPT_CFG "/usr/include/mrpt/mrpt-config")

include(${CMAKE_CURRENT_LIST_DIR}/../Algorithm/CMakeLists.txt)

include_directories(${SLAMBENCH_INCLUDE_DIR})
include_directories(${TOON_INCLUDE_PATHS})
include_directories(${CMAKE_CURRENT_LIST_DIR}/../Algorithm/src)
include_directories(
 ${MRPT_BASE}
 ${MRPT_CFG} 
)

add_library(${appname}-cuda  icsFusion_library.cpp )
SET_TARGET_PROPERTIES(${appname}-cuda PROPERTIES COMPILE_FLAGS "-fPIC")


target_link_libraries(${appname}-cuda  ${common_libraries} ics_fusion_cuda ${CUDA_LIBRARIES})
install(TARGETS ${appname}-cuda  DESTINATION  lib/)
generate_slam_library(${appname}-cuda "")



