cmake_minimum_required(VERSION 2.8.12)
project(point)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
add_definitions( ${PCL_DEFINITIONS} )
include_directories( ${PCL_INCLUDE_DIRS} )

add_executable(Pcd2Bev Pcd2Bev.cpp)
target_link_libraries(Pcd2Bev ${PCL_LIBRARIES} ${OpenCV_LIBS})

