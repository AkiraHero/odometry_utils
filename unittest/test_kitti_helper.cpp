//
// Created by xlju on 8/12/2021.
//
#include "kitti_helper.hpp"
#include "pcl/visualization/pcl_visualizer.h"
#include <thread>

int main() {
    auto helper = KittiHelper();
    std::string tst_file = "/home/xlju/data/kitti_odometry/dataset/sequences/00/velodyne/000000.bin";
    auto buf = helper.read_frm(tst_file);
    auto ptr = helper.buffer2cloud(buf);

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI> (ptr, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}