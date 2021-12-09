//
// Created by xlju on 9/12/2021.
//

#include "cal_pcl_feature.hpp"
#include "kitti_helper.hpp"

int main() {
    auto helper = KittiHelper();
    std::string tst_file = "/home/xlju/data/kitti_odometry/dataset/sequences/00/velodyne/000000.bin";
    auto buf = helper.read_frm(tst_file);
    auto ptr = helper.buffer2cloud(buf);
    auto data_results = get_data_for_sticky_pillar(ptr);

    return 0;
}

