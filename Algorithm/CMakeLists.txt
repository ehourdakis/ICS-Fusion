cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 OLD) #STOP stupid warnings
project(ics_fusion_cuda)

add_compile_options(-std=c++11)
set(G2O_USAGE, "USE_G2O")

#add_definitions(-DUSE_G2O)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder")
set(SUITESPARSE_INCLUDE_DIR "/usr/include/suitesparse")

# find_package(g2o REQUIRED REQUIRED )
# find_package(TooN   REQUIRED)
# find_package(Eigen3   REQUIRED)
# find_package(Thrust REQUIRED)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
set(TOON_INCLUDE_PATHS "$ENV{HOME}/src/TooN")

find_package(CUDA)
find_package(OpenCV)


# find_package(mrpt)
# set(MRPT_BASE "/usr/include/mrpt/base/include")
# set(MRPT_CFG "/usr/include/mrpt/mrpt-config")
# set(EIGEN3_INCLUDE_DIR "$ENV{HOME}/src/eigen")
# set(OPENCV_INCLUDE_DIR "/home/tavu/workspace/slambench2/deps/opencv/include/opencv2/")


LIST(APPEND G2O_LIBS
  cxsparse
  cholmod
  g2o_cli g2o_ext_freeglut_minimal g2o_simulator
  g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
  g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
  g2o_types_sba g2o_types_slam3d g2o_csparse_extension
  g2o_opengl_helper g2o_solver_dense g2o_stuff
  g2o_types_sclam2d g2o_parser g2o_solver_pcg
  g2o_types_data g2o_types_sim3
)

SET( CUDA_PROPAGATE_HOST_FLAGS OFF )
set(cuda_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/kernels.cu
    ${CMAKE_CURRENT_LIST_DIR}/src/icsfusion.cu
    ${CMAKE_CURRENT_LIST_DIR}/src/utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/kernelscalls.cu

)

set(cpp_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/volume.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/closeloop.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Isam.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/featuredetector.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/keyptsmap.cpp
    #${CMAKE_CURRENT_LIST_DIR}/src/g2oGraph.cpp    
    ${CMAKE_CURRENT_LIST_DIR}/src/saveData.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/smoothnetcore.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/smoothnet.cpp
)

include_directories(
  ./src
 ./src/cuda_headers
 ${TOON_INCLUDE_PATHS}
 ${MRPT_BASE}
 ${CSPARSE_INCLUDE_DIR}
 ${SUITESPARSE_INCLUDE_DIR}
#  ${MRPT_CFG}
)

cuda_add_library(ics_fusion_cuda_library   STATIC  ${cuda_srcs}  OPTIONS -std=c++11 -Xcompiler -fPIC -D_MWAITXINTRIN_H_INCLUDED -D_FORCE_INLINES
    -I${CMAKE_CURRENT_LIST_DIR}/src/cuda_headers
    -I${CMAKE_CURRENT_LIST_DIR}/src/
    -I${TOON_INCLUDE_PATHS}
    -I${EIGEN3_INCLUDE_DIR}
    -I${CSPARSE_INCLUDE_DIR}
    -I${SUITESPARSE_INCLUDE_DIR}
#    -DUSE_G2O
#     -I${MRPT_CFG}
#     -I${OPENCV_INCLUDE_DIR}
)



find_package( PCL 1.8 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )



SET_TARGET_PROPERTIES(ics_fusion_cuda_library PROPERTIES COMPILE_FLAGS "-fPIC" )
add_library(ics_fusion_cuda ${cpp_srcs})

target_compile_definitions(ics_fusion_cuda PRIVATE ${PCL_DEFINITIONS})

# if( PCL_FOUND )
#     include_directories( ${PCL_INCLUDE_DIRS} )
#     target_compile_definitions(ics_fusion_cuda PRIVATE ${PCL_DEFINITIONS})
#     target_link_libraries(ics_fusion_cuda ${PCL_LIBRARIES})
# endif()


SET_TARGET_PROPERTIES(ics_fusion_cuda PROPERTIES COMPILE_FLAGS "-fPIC" )
# target_link_libraries(ics_fusion_cuda ics_fusion_cuda_library ${CUDA_LIBRARIES} isam cholmod  ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(ics_fusion_cuda ics_fusion_cuda_library ${CUDA_LIBRARIES} isam cholmod  ${OpenCV_LIBS} )
target_link_libraries(ics_fusion_cuda ${PCL_LIBRARIES})



