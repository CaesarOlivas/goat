cmake_minimum_required(VERSION 3.5)

project(zed_projector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ZED 2 REQUIRED)
find_package(Threads REQUIRED)
find_package(PCL 1.8.1 REQUIRED COMPONENTS common)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(CATKIN_DEPENDS roscpp INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS} ${ZED_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
