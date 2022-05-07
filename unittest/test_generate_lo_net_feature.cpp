//
// Created by xlju on 20/12/2021.
//

#include "cal_lo_net_feature.hpp"
#include "boost/filesystem.hpp"

// binary format
// int H
// int W
// int C
// float[] H * W (x)
// float[] H * W (y)
// float[] H * W (z)
// float[] H * W (i)
// float[] H * W (r)
// float[] H * W (nx)
// float[] H * W (ny)
// float[] H * W (nz)

int main() {
    auto helper = KittiHelper();
    int squence_inx = 0;
    char squence_path[100];
    char file_name[100];
    std::string output_path = "/home/xlju/data/kitti_odometry/baseline_data/LONet/";

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
            auto range_img_map = generate_range_img(ptr);

            // write binary file
            char out_sub_dir[200];
            sprintf(out_sub_dir, "%02d/range_image/", squence_inx);
            boost::filesystem::create_directories(output_path + out_sub_dir);

            char outfile_feat[100];
            sprintf(outfile_feat, "%06d.rimg", frm_index);
            auto outfile_feat_path = output_path + out_sub_dir + outfile_feat;
            std::ofstream ofs_(outfile_feat_path, std::ios_base::binary);
            if(!ofs_.is_open()) {
                std::cout << "Cannot open file to write!" << std::endl;
                return -1;
            }
            int height = range_img_map["mat_x"].rows();
            int width = range_img_map["mat_x"].cols();
            int channels = range_img_map.size();
            ofs_.write(reinterpret_cast<char*>(&height), sizeof(int));
            ofs_.write(reinterpret_cast<char*>(&width), sizeof(int));
            ofs_.write(reinterpret_cast<char*>(&channels), sizeof(int));
            std::vector<std::string> keys = {
                    "mat_x",
                    "mat_y",
                    "mat_z",
                    "mat_i",
                    "mat_r",
                    "mat_nx",
                    "mat_ny",
                    "mat_nz"
            };
            for(auto const& key: keys) {
                auto const &mat = range_img_map[key];
                ofs_.write(reinterpret_cast<const char*>(mat.data()), sizeof(float) * mat.size());

            }
            frm_index += 1;
        }
        squence_inx += 1;
    }

}
