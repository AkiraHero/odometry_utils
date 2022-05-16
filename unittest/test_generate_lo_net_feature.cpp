//
// Created by xlju on 20/12/2021.
//

#include "cal_lo_net_feature.hpp"
#include "boost/filesystem.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>
#include <thread>
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

int main(int argc, char** argv) {
    assert(argc == 2);
    auto helper = KittiHelper();
    int squence_inx = atoi(argv[1]);
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
                sprintf(out_sub_dir, "%02d/xyz/", squence_inx);;
                boost::filesystem::create_directories(output_path + out_sub_dir);
                sprintf(out_sub_dir, "%02d/", squence_inx);;
            }

            char outfile_feat[100];
            sprintf(outfile_feat, "%06d.rimg", frm_index);

            char outfile_img[100];
            sprintf(outfile_img, "%06d.png", frm_index);

            int height = range_img_map["mat_x"].rows();
            int width = range_img_map["mat_x"].cols();

            // process coordinate x,y,z
            auto const &mat_x = range_img_map["mat_x"];
            auto const &mat_y = range_img_map["mat_y"];
            auto const &mat_z = range_img_map["mat_z"];
            float resolution = 0.005;
            assert(sizeof(ushort) == 2);
            Eigen::Matrix<ushort, Eigen::Dynamic, Eigen::Dynamic> new_mat_x = (mat_x / resolution).cast<ushort>();
            Eigen::Matrix<ushort, Eigen::Dynamic, Eigen::Dynamic> new_mat_y = (mat_y / resolution).cast<ushort>();
            Eigen::Matrix<ushort, Eigen::Dynamic, Eigen::Dynamic> new_mat_z = (mat_z / resolution).cast<ushort>();

            cv::Mat cvmat_x = cv::Mat::zeros(height, width, CV_16U);
            cv::Mat cvmat_y = cv::Mat::zeros(height, width, CV_16U);
            cv::Mat cvmat_z = cv::Mat::zeros(height, width, CV_16U);
            cv::eigen2cv(new_mat_x, cvmat_x);
            cv::eigen2cv(new_mat_y, cvmat_y);
            cv::eigen2cv(new_mat_z, cvmat_z);

            std::vector<cv::Mat> tmp;
            tmp.push_back(cvmat_x);
            tmp.push_back(cvmat_y);
            tmp.push_back(cvmat_z);
            cv::Mat cvmat_xyz;
            cv::merge(tmp, cvmat_xyz);

            auto img_file_xyz = output_path + out_sub_dir + "/xyz/" + outfile_img;
            cv::imwrite(img_file_xyz, cvmat_xyz);



            // process range and intensity
            auto const &mat_r = range_img_map["mat_r"];
            resolution = 0.002;
            assert(sizeof(ushort) == 2);
            Eigen::Matrix<ushort, Eigen::Dynamic, Eigen::Dynamic> new_mat = (mat_r / resolution).cast<ushort>();
            cv::Mat cvmat = cv::Mat::zeros(height, width, CV_16U);
            cv::eigen2cv(new_mat, cvmat);
            cv::Mat cvmat_r(height, width, CV_8UC2, cvmat.data);

            Eigen::MatrixXf mat_i = range_img_map["mat_i"] * 255.0f; // kitti intensity \in [0, 1]
            Eigen::Matrix<uchar , Eigen::Dynamic, Eigen::Dynamic> new_mat_i = mat_i.cast<uchar>();
            cv::Mat cvmat_i = cv::Mat::zeros(height, width, CV_8U);
            cv::eigen2cv(new_mat_i, cvmat_i);

            tmp.clear();
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
            // transfer [-1, 1] to [0, 1]
            Eigen::MatrixXf mat_nx_normalized = mat_nx ;
            mat_nx_normalized.array() += 1.f;
            mat_nx_normalized.array() /= 2.0f;

            Eigen::MatrixXf mat_ny_normalized = mat_ny ;
            mat_ny_normalized.array() += 1.f;
            mat_ny_normalized.array() /= 2.0f;

            Eigen::MatrixXf mat_nz_normalized = mat_nz ;
            mat_nz_normalized.array() += 1.f;
            mat_nz_normalized.array() /= 2.0f;


            cv::eigen2cv(mat_nx_normalized, cvmat_nx);
            cv::eigen2cv(mat_ny_normalized, cvmat_ny);
            cv::eigen2cv(mat_nz_normalized, cvmat_nz);
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
        break;
//        squence_inx += 1;
    }

}
