cmake_minimum_required(VERSION 3.20)
project(odometry_utils C CXX)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED)


include_directories(.
        ${PCL_INCLUDE_DIRS}
        )

#add_library(
#        odom_utils SHARED
#        kitti_helper.hpp
#)

set(LIBS ${PCL_LIBRARIES})



# unittests
add_executable(tst_kitti_helper
        unittest/test_kitti_helper.cpp
        )

target_link_libraries(
        tst_kitti_helper
        ${LIBS}
#        odom_utils
)