cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 OLD) #STOP stupid warnings
project(ics_fusion_cuda)

add_compile_options(-std=c++11)




set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder")




find_package( OpenCV REQUIRED )


set(TOON_INCLUDE_PATHS "$ENV{HOME}/src/TooN")



find_package(CUDA)
find_package(OpenCV)


SET( CUDA_PROPAGATE_HOST_FLAGS OFF ) 
set(cuda_srcs 
    ${CMAKE_CURRENT_LIST_DIR}/src/kernels.cu
    ${CMAKE_CURRENT_LIST_DIR}/src/icsfusion.cu
    ${CMAKE_CURRENT_LIST_DIR}/src/utils.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/volume.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/closeloop.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/Isam.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/featuredetector.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/keyptsmap.cpp
    #${CMAKE_CURRENT_LIST_DIR}/src/g2oGraph.cpp
)

include_directories(
  ./src
 ./src/cuda_headers
 ${TOON_INCLUDE_PATHS}
 ${MRPT_BASE}
)
# include_directories
cuda_add_library(ics_fusion_cuda   STATIC  ${cuda_srcs}  OPTIONS -std=c++11 -Xcompiler -fPIC -D_MWAITXINTRIN_H_INCLUDED -D_FORCE_INLINES 
    -I${CMAKE_CURRENT_LIST_DIR}/src/cuda_headers 
    -I${CMAKE_CURRENT_LIST_DIR}/src/ 
    -I${TOON_INCLUDE_PATHS}
    -I${EIGEN3_INCLUDE_DIR}
)

# target_include_directories(ics_fusion_cuda PRIVATE ${OPENCV_INCLUDE_DIR})
# include_directories(BEFORE SYSTEM ${OPENCV_INCLUDE_DIR})
#message(WARNING "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")

SET_TARGET_PROPERTIES(ics_fusion_cuda PROPERTIES COMPILE_FLAGS "-fPIC" )
target_link_libraries(ics_fusion_cuda ${CUDA_LIBRARIES} isam cholmod  ${OpenCV_LIBS})




