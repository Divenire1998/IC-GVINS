file(GLOB_RECURSE libGvinsSources CONFIGURE_DEPENDS
        src/*.cc
        include/*.h)
add_library(libIcGvinsCore STATIC
        ${libGvinsSources})
target_include_directories(libIcGvinsCore PUBLIC include ..)

# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 添加库 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

# OpenCV
find_package(OpenCV REQUIRED)
target_link_libraries(libIcGvinsCore ${OpenCV_LIBS})

# glog
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Glog REQUIRED)
target_link_libraries(libIcGvinsCore ${GLOG_LIBRARIES})

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(libIcGvinsCore ${YAML_CPP_LIBRARIES})

# Ceres
find_package(Ceres REQUIRED)
target_link_libraries(libIcGvinsCore ${CERES_LIBRARIES})

# boost
find_package(Boost REQUIRED COMPONENTS filesystem)
target_link_libraries(libIcGvinsCore
        ${Boost_LIBRARIES})

# TBB
find_package(TBB REQUIRED)
target_link_libraries(libIcGvinsCore
        TBB::tbb)

# abseil
target_link_libraries(libIcGvinsCore
        absl::strings
        absl::str_format
        absl::time)


# >>>>>>>>>>>>>>>>>>>> 添加库 >>>>>>>>>>>>>>>>>>>>>>>>>>>>