cmake_minimum_required(VERSION 3.31 FATAL_ERROR)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

project(camera_fusion)

find_package(OpenCV 4.11.0 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable (3D_object_tracking src/camFusion_Student.cpp src/main.cpp src/lidarData.cpp src/matching2D_Student.cpp src/objectDetection2D.cpp)
target_link_libraries (3D_object_tracking ${OpenCV_LIBRARIES})