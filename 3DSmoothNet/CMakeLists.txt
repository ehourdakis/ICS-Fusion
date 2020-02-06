cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 OLD) #STOP stupid warnings
project(3d_smooth_net_lib)

add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder")
set(SUITESPARSE_INCLUDE_DIR "/usr/include/suitesparse")

# include_directories( ${OpenCV_INCLUDE_DIRS} )
# set(TOON_INCLUDE_PATHS "$ENV{HOME}/src/TooN")

# find_package(CUDA)
# find_package(OpenCV)


# find_package(mrpt)
# set(MRPT_BASE "/usr/include/mrpt/base/include")
# set(MRPT_CFG "/usr/include/mrpt/mrpt-config")
# set(EIGEN3_INCLUDE_DIR "$ENV{HOME}/src/eigen")
# set(OPENCV_INCLUDE_DIR "/home/tavu/workspace/slambench2/deps/opencv/include/opencv2/")

find_package(CUDA)
set(cpp_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/smoothnet.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/smoothnetcore.cpp
)

include_directories(
  ./src
)

find_package( PCL 1.8 REQUIRED )

add_library(3d_smooth_net_lib ${cpp_srcs})
SET_TARGET_PROPERTIES(3d_smooth_net_lib PROPERTIES COMPILE_FLAGS "-fPIC" )


include_directories( ${PCL_INCLUDE_DIRS} )
target_compile_definitions(3d_smooth_net_lib PRIVATE ${PCL_DEFINITIONS})
# target_link_libraries(3d_smooth_net_lib ${PCL_LIBRARIES} ${CUDA_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(3d_smooth_net_lib ${PCL_LIBRARIES}  ${OpenCV_LIBS})
# target_link_libraries(3d_smooth_net_lib ${OpenCV_LIBS} ${PCL_LIBRARIES})




