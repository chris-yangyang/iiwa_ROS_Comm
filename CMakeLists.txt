cmake_minimum_required(VERSION 2.8.3)
project(iiwa_ROS_Comm)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cv_bridge image_transport)

find_package(OpenCV REQUIRED)
#find_package(VISP REQUIRED)

catkin_package()

include_directories(
include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)
#include_directories(${VISP_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(path_drawer src/main.cpp src/path_checker.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(path_drawer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
#target_link_libraries(path_drawer ${VISP_LIBRARIES})

target_link_libraries(path_drawer lapack blas)
