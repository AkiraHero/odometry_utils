//
// Created by xlju on 12/12/2021.
//
#include <thread>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include "pcl/visualization/pcl_visualizer.h"

#include <kitti_helper.hpp>

// kitti pose is in camera coordinate....

int main() {
    std::string pcl_1_file("/home/xlju/data/kitti_odometry/dataset/sequences/00/velodyne/000000.bin");
    std::string pcl_2_file("/home/xlju/data/kitti_odometry/dataset/sequences/00/velodyne/000003.bin");
    std::string pose_file_name = "/home/xlju/data/kitti_odometry/dataset/poses/00.txt";
    std::string calib_file = "/home/xlju/data/kitti_odometry/dataset/sequences/00/calib.txt";
    auto helper = KittiHelper();
    auto pose_results = helper.read_pose_file(pose_file_name);
    auto calib_results = helper.load_calib_file(calib_file);
    std::cout << "use calib Tr:" << std::endl;
    std::cout << calib_results["Tr"] << std::endl;

    auto lidar2camera = calib_results["Tr"];
    auto pose_1 = pose_results[0];
    auto pose_2 = pose_results[3];
    auto rel_pose = lidar2camera.inverse() * (pose_1.inverse() * pose_2) * lidar2camera;



    auto buf = helper.read_frm(pcl_1_file);
    auto ptr_1 = helper.buffer2cloud(buf);

    auto buf2 = helper.read_frm(pcl_2_file);
    auto ptr_2 = helper.buffer2cloud(buf2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_2_t(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*ptr_2, *ptr_2_t, rel_pose);


    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color1(ptr_1, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color2(ptr_1, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color2_t(ptr_2, 255, 255, 0);

    viewer->addPointCloud<pcl::PointXYZI> (ptr_1, single_color1, "pcl1");
    viewer->addPointCloud<pcl::PointXYZI> (ptr_2, single_color2, "pcl2");
    viewer->addPointCloud<pcl::PointXYZI> (ptr_2_t, single_color2_t, "pcl2_t");

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}