cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
find_package( Eigen3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=deprecated-declarations ") # Eigen raises deprecated declarations warning.
include_directories(
 ${EIGEN3_INCLUDE_DIR}
)

add_executable(isamTestSimple main_simple.cpp)
target_link_libraries(isamTestSimple /usr/local/lib/libisam.a cholmod)



