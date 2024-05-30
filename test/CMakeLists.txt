add_executable(show_path show_path.cpp)
target_link_libraries(show_path 
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_executable(vis_bag vis_bag.cpp)
target_link_libraries(vis_bag 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
)

add_executable(check_imu check_imu.cpp)
target_link_libraries(check_imu 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
)