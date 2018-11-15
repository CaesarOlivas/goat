cmake_minimum_required(VERSION 3.5)

project(zed_projector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(CUDA 9 REQUIRED)
find_package(ZED 2.7 REQUIRED)
find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${ZED_INCLUDE_DIRS}
					${OpenCV_INCLUDE_DIRS})

set(SOURCES src/errors.cpp src/posix_clock.cpp src/projector.cpp src/zed_utils.cpp)

add_executable(test_projector src/test_projector.cpp ${SOURCES})
target_link_libraries(test_projector ${catkin_LIBRARIES} ${CUDA_LIBRARIES} ${ZED_LIBRARIES}
					  ${OpenCV_LIBRARIES})
