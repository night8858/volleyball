# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.1.0)

project(Volleyball_Vision)

# Find librealsense2 installed package
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( "/usr/include/eigen3" ) 
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Add the application sources to the target
add_executable(Volleyball_Vision Volleyball_Vision.cpp)

# Link librealsense2 and OpenCV to the target
target_link_libraries(Volleyball_Vision PRIVATE ${realsense2_LIBRARY} ${OpenCV_LIBS})
