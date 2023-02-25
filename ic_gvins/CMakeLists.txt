cmake_minimum_required(VERSION 3.10)

# C++ 17 support
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ic_gvins)

# 默认使用Release模式
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

# 编译选项
add_compile_options(-fPIC)

# ROS支持
find_package(catkin REQUIRED COMPONENTS
        message_generation
        roscpp
        std_msgs)
catkin_package(
        LIBRARIES
        CATKIN_DEPENDS roscpp std_msgs)
include_directories(
        ${catkin_INCLUDE_DIRS})

# Gvins lib
add_subdirectory(libIcGvins)

# ROS 接口调用
add_subdirectory(example)

# abseil-cpp lib
set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(thirdparty/abseil-cpp abseil-cpp)