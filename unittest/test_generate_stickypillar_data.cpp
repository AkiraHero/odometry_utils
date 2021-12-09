//
// Created by xlju on 9/12/2021.
//

#include <fstream>

#include "cal_pcl_feature.hpp"
#include "kitti_helper.hpp"


/* saving format:
 * int pilla num
 * int pilla feature num N
 * double[11] * N: features...
 *
 *
 */

int main() {
    auto helper = KittiHelper();
    int squence_inx = 0;
    char squence_path[100];
    char file_name[100];
    std::string output_path = "/home/xlju/data/kitti_odometry/baseline_data/stickypillar/";

    while(true) {
        sprintf(squence_path, "/home/xlju/data/kitti_odometry/dataset/sequences/%02d/velodyne/", squence_inx);
        int frm_index = 0;
        while(true) {
            sprintf(file_name, "%06d.bin", frm_index);
            auto file_path = std::string(squence_path) + file_name;
            std::cout << "Processing file:" << file_path << std::endl;
            std::ifstream fs(file_path);
            if (!fs.is_open()) {
                if(frm_index == 0) {
                    std::cout << "Process over!" << std::endl;
                    return 0;
                }
                break;
            }
            auto buf = helper.read_frm(file_path);
            auto ptr = helper.buffer2cloud(buf);
            auto data_results = get_data_for_sticky_pillar(ptr);

            // write binary file
            char outfile[100];
            sprintf(outfile, "pillar_feature_seq%02d_%06d.feat", squence_inx, frm_index);
            auto outfile_path = output_path + outfile;
            std::ofstream ofs_(outfile_path, std::ios_base::binary);
            if(!ofs_.is_open()) {
                std::cout << "Cannot open file to write!" << std::endl;
                return -1;
            }
            int pillar_num = data_results.size();
            ofs_.write(reinterpret_cast<char*>(&pillar_num), sizeof(int));
            for(auto const& c: data_results) {
                int feature_num = c.size();
                ofs_.write(reinterpret_cast<char*>(&feature_num), sizeof(int));
                for(auto const& ff: c) {
                    ofs_.write(reinterpret_cast<const char*>(ff.data()), sizeof(double) * ff.size());
                }
            }
            frm_index += 1;
        }
        squence_inx += 1;
    }

}

