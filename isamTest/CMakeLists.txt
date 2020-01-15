cmake_minimum_required(VERSION 2.8.3)

find_package( Eigen3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations ") # Eigen raises deprecated declarations warning.
include(${CMAKE_CURRENT_LIST_DIR}/Algorithm/CMakeLists.txt)

include_directories(${CMAKE_CURRENT_LIST_DIR}/Algorithm/src)
include_directories(
 ${MRPT_BASE}
 ${MRPT_CFG} 
 ${EIGEN3_INCLUDE_DIR}
)

add_executable(isamTest main.cpp)
add_executable(isamTestSimple main_simple.cpp)

target_link_libraries(isamTestSimple /usr/local/lib/libisam.a ics_fusion_cuda ${CUDA_LIBRARIES} )
target_link_libraries(isamTest /usr/local/lib/libisam.a ics_fusion_cuda ${CUDA_LIBRARIES} )



