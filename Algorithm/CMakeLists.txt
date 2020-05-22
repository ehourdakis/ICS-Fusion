cmake_minimum_required(VERSION 3.3)
project(ics_fusion_lib)

find_package(CUDA)

#find_package(open3d)
#find_package(qhull)

add_compile_options(-std=c++14 -fopenmp)
# add_definitions(-DUSE_G2O)

option(USE_OPEN3D "Use open3d library" OFF)
set(USE_OPEN3D OFF)

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

SET(sift_src
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/CovEstArgumentEvaluator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/CovEstimator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/covUtils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/GeneralFeature.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/ImagePyramid.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/imgfeatures.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/ipoint.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/sift.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/sift_cov.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/sift_cov.h
    ${CMAKE_CURRENT_LIST_DIR}/src/sift/utils.cpp
)

SET(teaser_src
    ${CMAKE_CURRENT_LIST_DIR}/src/teaser/graph.cc
    ${CMAKE_CURRENT_LIST_DIR}/src/teaser/matcher.cc
    ${CMAKE_CURRENT_LIST_DIR}/src/teaser/registration.cc
)

SET(pmc_src
    ${CMAKE_CURRENT_LIST_DIR}/src/pmc/pmc_clique_utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/pmc/pmc_cores.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/pmc/pmc_graph.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/pmc/pmc_heu.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/pmc/pmc_lib.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/pmc/pmc_maxclique.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/pmc/pmc_utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/pmc/pmcx_maxclique_basic.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/pmc/pmcx_maxclique.cpp
)


set(TOON_INCLUDE_PATHS "${CMAKE_CURRENT_LIST_DIR}/dependencies/TooN")
set(EIGEN_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/dependencies/eigen")
set(CUDA_INCLUDE_DIRS "/usr/local/cuda/include")
#set(SUITESPARSE_INCLUDE_DIR "/usr/include/suitesparse")
#set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.7")

set(eigen3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} )
find_package( OpenCV REQUIRED )

include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${TOON_INCLUDE_PATHS}
    ${EIGEN_INCLUDE_DIRS}
    ${SUITESPARSE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}  
    ${CMAKE_CURRENT_LIST_DIR}/src/teaser
    ${CMAKE_CURRENT_LIST_DIR}/src/pmc
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
    ${CMAKE_CURRENT_LIST_DIR}/src/Isam.cpp     
    ${CMAKE_CURRENT_LIST_DIR}/src/saveData.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/harris.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/SiftCovEstimator.cpp
)

if(${USE_OPEN3D})
    set(OPEN3D_DEF "USE_OPEN3D")
else()
    set(OPEN3D_DEF "NOT_USE_OPEN3D")
endif()

SET( CUDA_PROPAGATE_HOST_FLAGS OFF )

find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};-Xcompiler;-fPIC;" )
cuda_add_library(ics_fusion_cuda_lib   STATIC  ${cuda_srcs}  OPTIONS -std=c++11; -Xcompiler; -fPIC; 
    -D_MWAITXINTRIN_H_INCLUDED -D_FORCE_INLINES
    -I${CMAKE_CURRENT_LIST_DIR}/src/
    -I${TOON_INCLUDE_PATHS}
    -I${EIGEN3_INCLUDE_DIR}
)
SET_TARGET_PROPERTIES(ics_fusion_cuda_lib PROPERTIES COMPILE_FLAGS "-fPIC" )

add_library(ics_fusion_lib 
    ${cpp_srcs}
    ${sift_src}
    ${teaser_src}
    ${pmc_src}
)

message(WARNING ${OPEN3D_DEF})

if(${USE_OPEN3D})
    SET_TARGET_PROPERTIES(ics_fusion_lib PROPERTIES COMPILE_FLAGS "-fPIC -fopenmp -DUSE_OPEN3D -ggdb" )
else()
    SET_TARGET_PROPERTIES(ics_fusion_lib PROPERTIES COMPILE_FLAGS "-fPIC -fopenmp" )
endif()
    
find_package(OpenMP)
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

#Comment ISAM if use G2O
target_link_libraries(ics_fusion_lib
    ${OpenCV_LIBS} 
#    ${CUDA_LIBRARIES}    
    ics_fusion_cuda_lib
    isam
    cholmod
)

if(${USE_OPEN3D})
    target_link_libraries(ics_fusion_lib Open3D.a qhull libqhullcpp.a)
endif()
