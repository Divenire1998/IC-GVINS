cmake_minimum_required(VERSION 3.5)
project(kaist_bag_fix)

set(CMAKE_CXX_STANDARD 11)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# catkin macros and libraries
find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        irp_sen_msgs
        rosbag)

include_directories(
        ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cc)

# ROS
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES})

# glog
find_package(glog REQUIRED)
target_link_libraries(${PROJECT_NAME} glog::glog)

# OpenCV
find_package(OpenCV REQUIRED)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# abseil
find_package(absl REQUIRED)
target_link_libraries(${PROJECT_NAME}
        absl::strings
        absl::str_format)
