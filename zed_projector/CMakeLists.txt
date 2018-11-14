cmake_minimum_required(VERSION 3.5)

project(zed_projector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(CUDA 9 REQUIRED)
find_package(ZED 2.7 REQUIRED)
find_package(Threads REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS}
					${ZED_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

set(SOURCES src/errors.cpp src/posix_clock.cpp src/projector.cpp)

add_executable(test_projector src/test_projector.cpp ${SOURCES})
