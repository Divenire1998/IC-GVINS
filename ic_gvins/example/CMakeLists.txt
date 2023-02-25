
# ROS 例子
add_executable(${PROJECT_NAME}_ros
        ROS/fusion_ros.cc
        ROS/drawer_rviz.cc)

add_dependencies(${PROJECT_NAME}_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_ros
        ${catkin_LIBRARIES}
        libIcGvinsCore)


# 读取
add_executable(fusion_raw
        ROS/fusion_raw.cc
        ROS/drawer_rviz.cc)

add_dependencies(fusion_raw
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS})
target_link_libraries(fusion_raw
        ${catkin_LIBRARIES}
        libIcGvinsCore)