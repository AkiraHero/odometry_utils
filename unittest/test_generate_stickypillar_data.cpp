//
// Created by xlju on 13/12/2021.
//

#include <fstream>

#include "cal_pcl_feature.hpp"
#include "kitti_helper.hpp"
#include "boost/filesystem.hpp"

/* feature saving format:
 * int pilla num
 * int pilla feature num N
 * float[11] * N: features...
 *
 * pt saving format
 * int pilla num N
 * float[4] * N
 *
 */

int main() {
    auto helper = KittiHelper();
    int squence_inx = 0;
    char squence_path[100];
    char file_name[100];
    std::string output_path = "/home/xlju/data/kitti_odometry/baseline_data/hh/";

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
            std::vector<pcl::PointXYZI> pillar_key_points;
            auto data_results = get_data_for_sticky_pillar(ptr, pillar_key_points);

            // write binary file
            char out_sub_dir[200];
            sprintf(out_sub_dir, "%02d/pillar_feature/", squence_inx);
            boost::filesystem::create_directories(output_path + out_sub_dir);

            char out_sub_dir_pt[200];
            sprintf(out_sub_dir_pt, "%02d/pillar_keypoint/", squence_inx);
            boost::filesystem::create_directories(output_path + out_sub_dir_pt);

            char outfile_pillar_points[100];
            sprintf(outfile_pillar_points, "%06d.pt", frm_index);
            auto outfile_pt_path = output_path + out_sub_dir_pt + outfile_pillar_points;
            std::ofstream ofs_pt_(outfile_pt_path, std::ios_base::binary);
            if(!ofs_pt_.is_open()) {
                std::cout << "Cannot open file to write!" << std::endl;
                return -1;
            }
            int pillar_pt_size = pillar_key_points.size();
            ofs_pt_.write(reinterpret_cast<const char*>(&pillar_pt_size), sizeof(int));
            for(auto const& c: pillar_key_points) {
                const float &x = c.x;
                const float &y = c.y;
                const float &z = c.z;
                const float &intensity = c.intensity;
                ofs_pt_.write(reinterpret_cast<const char*>(&x), sizeof(float));
                ofs_pt_.write(reinterpret_cast<const char*>(&y), sizeof(float));
                ofs_pt_.write(reinterpret_cast<const char*>(&z), sizeof(float));
                ofs_pt_.write(reinterpret_cast<const char*>(&intensity), sizeof(float));
            }

            char outfile_feat[100];
            sprintf(outfile_feat, "%06d.feat", frm_index);
            auto outfile_feat_path = output_path + out_sub_dir + outfile_feat;
            std::ofstream ofs_(outfile_feat_path, std::ios_base::binary);
            if(!ofs_.is_open()) {
                std::cout << "Cannot open file to write!" << std::endl;
                return -1;
            }
            int pillar_num = data_results.size();
            ofs_.write(reinterpret_cast<char*>(&pillar_num), sizeof(int));
            for(int i = 0; i != data_results.size(); i++) {
                auto const &c = data_results[i];
                int feature_num = c.size();
                ofs_.write(reinterpret_cast<char*>(&feature_num), sizeof(int));
                std::cout << "sizeof(float)" << sizeof(float) << std::endl;
                std::cout << "sizeof(int)" << sizeof(int) << std::endl;

                for(auto const& ff: c) {
                    ofs_.write(reinterpret_cast<const char*>(ff.data()), sizeof(float) * ff.size());
                }
            }
            frm_index += 1;
        }
        squence_inx += 1;
    }

}

