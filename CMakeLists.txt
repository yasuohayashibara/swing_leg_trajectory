cmake_minimum_required(VERSION 2.8)

project(ViaPoint)

set(CMAKE_CXX_FLAGS_RELEASE)
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR} CACHE PATH "Eigen installation dir(Default: $EIGEN_HOME)")
add_definitions(-lstdc++ -std=c++11 -w)

include_directories(
	${EIGEN3_INCLUDE_DIR}
)

add_executable(ViaPoint main.cpp ViaPoint.cpp ViaPoint.h)
