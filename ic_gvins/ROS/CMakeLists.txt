add_executable(${PROJECT_NAME}_ros
        fusion_ros.cc
        drawer_rviz.cc)

add_dependencies(${PROJECT_NAME}_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_ros
        ${catkin_LIBRARIES}
        libIcGvinsCore)