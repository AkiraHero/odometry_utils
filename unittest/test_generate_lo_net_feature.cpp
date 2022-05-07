//
// Created by xlju on 20/12/2021.
//

#include "cal_lo_net_feature.hpp"
#include "boost/filesystem.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>
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
    std::string output_path = "/data/LONet_data/";

    while(true) {
        sprintf(squence_path, "/data/dataset/sequences/%02d/velodyne/", squence_inx);
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
            if(frm_index == 0) {
                sprintf(out_sub_dir, "%02d/range_intensity/", squence_inx);;
                boost::filesystem::create_directories(output_path + out_sub_dir);
                sprintf(out_sub_dir, "%02d/normal/", squence_inx);;
                boost::filesystem::create_directories(output_path + out_sub_dir);
                sprintf(out_sub_dir, "%02d/", squence_inx);;
            }

            char outfile_feat[100];
            sprintf(outfile_feat, "%06d.rimg", frm_index);

            char outfile_img[100];
            sprintf(outfile_img, "%06d.png", frm_index);


//            auto outfile_feat_path = output_path + out_sub_dir + outfile_feat;
//            std::ofstream ofs_(outfile_feat_path, std::ios_base::binary);
//            if(!ofs_.is_open()) {
//                std::cout << "Cannot open file to write!" << std::endl;
//                return -1;
//            }
            int height = range_img_map["mat_x"].rows();
            int width = range_img_map["mat_x"].cols();
//            int channels = range_img_map.size();
//            ofs_.write(reinterpret_cast<char*>(&height), sizeof(int));
//            ofs_.write(reinterpret_cast<char*>(&width), sizeof(int));
//            ofs_.write(reinterpret_cast<char*>(&channels), sizeof(int));
//            std::vector<std::string> keys = {
//                    "mat_x",
//                    "mat_y",
//                    "mat_z",
//                    "mat_i",
//                    "mat_r",
//                    "mat_nx",
//                    "mat_ny",
//                    "mat_nz"
//            };
            // process range and intensity
            auto const &mat_r = range_img_map["mat_r"];
            float resolution = 0.002;
            assert(sizeof(ushort) == 2);
            Eigen::Matrix<ushort, Eigen::Dynamic, Eigen::Dynamic> new_mat = (mat_r / resolution).cast<ushort>();
            cv::Mat cvmat = cv::Mat::zeros(height, width, CV_16U);
            cv::eigen2cv(new_mat, cvmat);
            cv::Mat cvmat_r(height, width, CV_8UC2, cvmat.data);

            auto const &mat_i = range_img_map["mat_i"];
            Eigen::Matrix<uchar , Eigen::Dynamic, Eigen::Dynamic> new_mat_i = mat_i.cast<uchar>();
            cv::Mat cvmat_i = cv::Mat::zeros(height, width, CV_8U);
            cv::eigen2cv(new_mat_i, cvmat_i);

            std::vector<cv::Mat> tmp;
            tmp.push_back(cvmat_r);
            tmp.push_back(cvmat_i);
            cv::Mat cvmat_ri;
            cv::merge(tmp, cvmat_ri);

            auto img_file_ri = output_path + out_sub_dir + "/range_intensity/" + outfile_img;
            cv::imwrite(img_file_ri, cvmat_ri);

            // process nx.ny.nz

            auto const &mat_nx = range_img_map["mat_nx"];
            auto const &mat_ny = range_img_map["mat_ny"];
            auto const &mat_nz = range_img_map["mat_nz"];
            cv::Mat cvmat_nx = cv::Mat::zeros(height, width, CV_32F);
            cv::Mat cvmat_ny = cv::Mat::zeros(height, width, CV_32F);
            cv::Mat cvmat_nz = cv::Mat::zeros(height, width, CV_32F);
            cv::eigen2cv(mat_nx, cvmat_nx);
            cv::eigen2cv(mat_ny, cvmat_ny);
            cv::eigen2cv(mat_nz, cvmat_nz);
            tmp.clear();
            tmp.push_back(cvmat_nx);
            tmp.push_back(cvmat_ny);
            tmp.push_back(cvmat_nz);
            cv::Mat cvmat_normal;
            cv::merge(tmp, cvmat_normal);
            cv::Mat cvmat_normal_255;
            cvmat_normal.convertTo(cvmat_normal_255, CV_8UC3, 255);

            auto img_file_normal = output_path + out_sub_dir + "/normal/" + outfile_img;
            cv::imwrite(img_file_normal, cvmat_normal_255);

            frm_index += 1;
        }
        squence_inx += 1;
    }

}
