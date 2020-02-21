cmake_minimum_required(VERSION 3.3)
project(ics_fusion_lib)

find_package(CUDA)
add_compile_options(-std=c++11)
#add_definitions(-DUSE_G2O)
LIST(APPEND G2O_LIBS
  cxsparse  
  g2o_cli g2o_ext_freeglut_minimal g2o_simulator
  g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
  g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
  g2o_types_sba g2o_types_slam3d g2o_csparse_extension
  g2o_opengl_helper g2o_solver_dense g2o_stuff
  g2o_types_sclam2d g2o_parser g2o_solver_pcg
  g2o_types_data g2o_types_sim3
)


set(TOON_INCLUDE_PATHS "$ENV{HOME}/src/TooN")
# set(TOON_INCLUDE_PATHS "/usr/local/src/TooN")
set(EIGEN_INCLUDE_DIRS "/usr/local/include/eigen3")
set(CUDA_INCLUDE_DIRS "/usr/local/cuda/include")
set(SUITESPARSE_INCLUDE_DIR "/usr/include/suitesparse")
set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.7")

find_package( OpenCV REQUIRED )
 

include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${TOON_INCLUDE_PATHS}
    ${EIGEN_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

set(cuda_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/kernels.cu
    ${CMAKE_CURRENT_LIST_DIR}/src/icsfusion.cu
    ${CMAKE_CURRENT_LIST_DIR}/src/utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/kernelscalls.cu

)

set(cpp_srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/volume.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/closeloop.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/featuredetector.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/keyptsmap.cpp
    #${CMAKE_CURRENT_LIST_DIR}/src/g2oGraph.cpp   
    ${CMAKE_CURRENT_LIST_DIR}/src/Isam.cpp     
    ${CMAKE_CURRENT_LIST_DIR}/src/saveData.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/harris.cpp
#     ${CMAKE_CURRENT_LIST_DIR}/src/smoothnetcore.cpp
#     ${CMAKE_CURRENT_LIST_DIR}/src/smoothnet.cpp

)


SET( CUDA_PROPAGATE_HOST_FLAGS OFF )

find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};-Xcompiler;-fPIC;" )
cuda_add_library(ics_fusion_cuda_lib   STATIC  ${cuda_srcs}  OPTIONS -std=c++11; -Xcompiler; -fPIC; -D_MWAITXINTRIN_H_INCLUDED -D_FORCE_INLINES
    -I${CMAKE_CURRENT_LIST_DIR}/src/
    -I${TOON_INCLUDE_PATHS}
    -I${EIGEN3_INCLUDE_DIR}
)
SET_TARGET_PROPERTIES(ics_fusion_cuda_lib PROPERTIES COMPILE_FLAGS "-fPIC" )




add_library(ics_fusion_lib 
    ${cpp_srcs}
)

add_compile_options(-ggdb)
SET_TARGET_PROPERTIES(ics_fusion_lib PROPERTIES COMPILE_FLAGS "-fPIC" )


#Comment ISAM if use G2O
target_link_libraries(ics_fusion_lib 
#    ${G2O_LIBS}
    ${OpenCV_LIBS} 
    ${CUDA_LIBRARIES}
    ics_fusion_cuda_lib
    isam
    cholmod
)

# add_dependencies(talker node_example_gencfg node_example_generate_messages_cpp)

