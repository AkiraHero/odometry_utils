//
// Created by xlju on 8/12/2021.
//
#include "kitti_helper.hpp"

int main() {
    auto helper = KittiHelper();
    std::string tst_file = "/home/xlju/data/kitti_odometry/dataset/sequences/00/velodyne/000000.bin";
    auto ptr = helper.read_frm(tst_file);
    std::cout << "Load points num:" << ptr->size() << std::endl;
    return 0;
}