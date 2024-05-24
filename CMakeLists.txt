cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

project(sdpc)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    livox_ros_driver
    rosbag
)

find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(Boost REQUIRED COMPONENTS
#     filesystem
# )

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

# add_library(sdpc SHARED src/distortpc.cpp)
# target_include_directories(sdpc PRIVATE
#     ${PCL_INCLUDE_DIRS}
# )
# target_link_libraries(sdpc 
#     ${catkin_LIBRARIES} 
#     ${GAZEBO_LIBRARIES}
#     ${PCL_LIBRARIES}
# )


add_executable(mld src/mld.cpp)
target_include_directories(mld PRIVATE
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(mld
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    gazebo_rendering
    ${PCL_LIBRARIES}
    yaml-cpp
)
target_compile_definitions(mld PUBLIC -DROOT_DIR=\"${CMAKE_SOURCE_DIR}\")

add_executable(_test_mld src/_test_mld.cpp)
target_include_directories(_test_mld PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)
target_link_libraries(_test_mld
    ${GTEST_LIBRARIES}
    ${catkin_LIBRARIES}
    gazebo_test_fixture
    ${GAZEBO_LIBRARIES}
    ${PCL_LIBRARIES}
)
target_compile_definitions(_test_mld PUBLIC -DROOT_DIR=\"${CMAKE_SOURCE_DIR}\")

add_subdirectory(test)

add_executable(_test_cam src/_test_cam.cpp)
target_include_directories(_test_cam PRIVATE
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(_test_cam
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    yaml-cpp
)