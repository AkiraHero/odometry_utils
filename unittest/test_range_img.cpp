//
// Created by xlju on 22/12/2021.
//
#include "kitti_helper.hpp"
#include "cal_lo_net_feature.hpp"
#include "opencv/cv.hpp"
#include <cassert>
#include <chrono>


int main() {
    auto helper = KittiHelper();
    int frm_inx = 0;
    char frm_file[100];
    while(1) {
        std::string tst_file = "/data/dataset/sequences/00/velodyne/";
        sprintf(frm_file, "%06d.bin", frm_inx);
        tst_file = tst_file + frm_file;
        std::cout << "reading file:" << tst_file << std::endl;
        auto buf = helper.read_frm(tst_file);
        auto ptr = helper.buffer2cloud(buf);

//        auto st = std::chrono::high_resolution_clock::now();
        auto range_img_map = generate_range_img(ptr);
//        auto ed = std::chrono::high_resolution_clock::now();
//        auto time_diff = ed - st;
//        std::cout << "time cost1:" << time_diff.count() << std::endl;


        auto cols = range_img_map["mat_r"].cols();
        auto rows = range_img_map["mat_r"].rows();
        cv::Mat range_img(rows, cols, CV_8UC3);
        range_img.setTo(0);

        float max_range = 30.0f;
        float min_range = 2.0f;

        for(int i = 0; i != rows; i++) {
            for(int j = 0; j != cols; j++) {
                auto range = range_img_map["mat_r"](i, j);
                if(range != 0) {
                    range = std::min(range, max_range);
                    range = std::max(range, min_range);
                    unsigned char b = (range - min_range) / (max_range - min_range) * 255.0f;
                    unsigned char g = b < 128? b : 255 - b;
                    unsigned char r = 145;
                    range_img.at<cv::Vec3b>(i, j) = cv::Vec3b(b, g, r);
                }
            }
        }
        cv::resize(range_img, range_img, cv::Size(2000, 300));

        // draw normal
        auto nx = range_img_map["mat_nx"];
        auto ny = range_img_map["mat_ny"];
        auto nz = range_img_map["mat_nz"];

        cols = range_img_map["mat_nx"].cols();
        rows = range_img_map["mat_nx"].rows();
        cv::Mat normal_img(rows, cols, CV_8UC3);
        normal_img.setTo(0);


        for(int i = 0; i != rows; i++) {
            for(int j = 0; j != cols; j++) {
                auto normal_x = nx(i, j);
                auto normal_y = ny(i, j);
                auto normal_z = nz(i, j);


                if(normal_x != 0) {
                    unsigned char b = (normal_x + 1) / 2.0f * 255;
                    unsigned char g = (normal_y + 1) / 2.0f * 255;
                    unsigned char r = (normal_z + 1) / 2.0f * 255;
                    normal_img.at<cv::Vec3b>(i, j) = cv::Vec3b(b, g, r);
                }
            }
        }
        cv::resize(normal_img, normal_img, cv::Size(2000, 300));

        cv::flip(range_img, range_img, 0);
        cv::flip(normal_img, normal_img, 0);
        cv::flip(range_img, range_img, 1);
        cv::flip(normal_img, normal_img, 1);
        cv::imshow("range_img", range_img);
        cv::imshow("normal_img", normal_img);
        cv::waitKey(1);
        frm_inx += 1;
//        auto ed2 = std::chrono::high_resolution_clock::now();
//        auto time_diff2 = ed2 - ed;
//        std::cout << "time cost2:" << time_diff2.count() << std::endl;
    }
    return 0;
}