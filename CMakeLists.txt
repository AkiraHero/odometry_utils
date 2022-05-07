cmake_minimum_required(VERSION 3.20)
project(odometry_utils C CXX)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED)
find_package(OpenCV 3.4.13 EXACT REQUIRED PATHS /home/xlju/software/opencv-3.4.13/install
        )
find_package(
        Eigen3 REQUIRED
)


include_directories(.
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS})

#add_library(
#        odom_utils SHARED
#        kitti_helper.hpp
#)

set(LIBS
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )



# unittests
add_executable(tst_kitti_helper
        unittest/test_kitti_helper.cpp
        )

target_link_libraries(
        tst_kitti_helper
        ${LIBS}
#        odom_utils
)

add_executable(tst_pcl_processing
        unittest/test_pcl_processing.cpp
        )
target_link_libraries(
        tst_pcl_processing
        ${LIBS}
        #        odom_utils
)

add_executable(gen_sticky_pillar_data
        unittest/test_generate_stickypillar_data.cpp
)
target_link_libraries(
        gen_sticky_pillar_data
        ${LIBS}
)

add_executable(test_pose_file_reading
        unittest/test_posefile_reading.cpp
        )

add_executable(test_pcl_transform
        unittest/test_pcl_transform.cpp
        )

target_link_libraries(
        test_pcl_transform
        ${LIBS}
)

add_executable(
        test_gen_pillar_correspondence
        unittest/test_generate_pillar_correspondence.cpp
)
target_link_libraries(
        test_gen_pillar_correspondence
        ${LIBS}
)

add_executable(test_generate_lo_net_feature
        unittest/test_generate_lo_net_feature.cpp
)

target_link_libraries(
        test_generate_lo_net_feature
        ${LIBS}
)

add_executable(test_neighbour_generator
        unittest/test_neighbour_generator.cpp
        )

target_link_libraries(
        test_neighbour_generator
        ${LIBS}
)

add_executable(test_range_img
        unittest/test_range_img.cpp
        )

target_link_libraries(
        test_range_img
        ${LIBS}
)