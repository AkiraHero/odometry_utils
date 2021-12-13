//
// Created by xlju on 12/12/2021.
//

#include "kitti_helper.hpp"

int main() {
    std::string filename = "/home/xlju/data/kitti_odometry/dataset/poses/00.txt";
    auto helper = KittiHelper();
    auto results = helper.read_pose_file(filename);
    std::cout << "get matrices num:" << results.size() << std::endl;
    return 0;
}