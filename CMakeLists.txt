cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 17)

project(sdpc)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    livox_ros_driver
    rosbag
)

find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sdpc
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

add_library(sdpc SHARED src/distortpc.cpp)
target_include_directories(sdpc PRIVATE
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(sdpc 
    ${catkin_LIBRARIES} 
    ${GAZEBO_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_subdirectory(test)
