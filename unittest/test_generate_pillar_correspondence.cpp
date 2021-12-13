//
// Created by xlju on 13/12/2021.
//


/*
 * save matching matrix
 * file format:
 * int rows
 * int cols
 * col1...
 * col2...
 * ...
 *
 */


#include <fstream>
#include <cassert>

#include "cal_pcl_feature.hpp"
#include "kitti_helper.hpp"
#include "boost/filesystem.hpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr read_pillar_point_file(std::string file_name) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_res(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream fs(file_name);
    assert (fs.is_open());
    int point_num = 0;
    fs.read(reinterpret_cast<char*>(&point_num), sizeof(int));
    float tmp[4];
    for (int i = 0; i != point_num; i++) {
        pcl::PointXYZ pt;
        fs.read(reinterpret_cast<char*>(tmp), sizeof(float ) * 4);
        assert(fs.good());
        pt.x = tmp[0];
        pt.y = tmp[1];
        pt.z = tmp[2];
        pcl_res->push_back(pt);
    }
    return pcl_res;
}

bool check_file_existence(std::string file) {
    std::ifstream fs(file);
    return fs.is_open();
}



int main() {
    auto helper = KittiHelper();
    int frm_delta = 1;
    int squence_inx = 0;
    char pillar_point_path[100];
    char file_name[100];
    char pose_file[100];
    std::string output_path = "/home/xlju/data/kitti_odometry/baseline_data/stickypillar/";
    float radius = 0.5f;

    while(true) {
        auto helper = KittiHelper();
        sprintf(pose_file, "/home/xlju/data/kitti_odometry/dataset/poses/%02d.txt", squence_inx);
        auto poselist = helper.read_pose_file(pose_file);
        sprintf(pillar_point_path, "/home/xlju/data/kitti_odometry/baseline_data/stickypillar/%02d/pillar_keypoint/", squence_inx);
        char calib_file[100];
        sprintf(calib_file,  "/home/xlju/data/kitti_odometry/dataset/sequences/%02d/calib.txt", squence_inx);
        auto calib = helper.load_calib_file(calib_file);

        auto lidar2camera = calib["Tr"];




        int frm_index = 0;
        while(true) {
            sprintf(file_name, "%06d.pt", frm_index);
            auto ref_file_path = std::string(pillar_point_path) + file_name;
            sprintf(file_name, "%06d.pt", frm_index + frm_delta);
            auto tar_file_path = std::string(pillar_point_path) + file_name;
            if(!check_file_existence(tar_file_path)) {
                break;
            }
            std::cout << "Processing file:" << ref_file_path << std::endl;
            auto ref_pcl = read_pillar_point_file(ref_file_path);
            auto tar_pcl = read_pillar_point_file(tar_file_path);

            auto pose_ref = poselist[frm_index];
            auto pose_tar = poselist[frm_index + frm_delta];
            auto rel_pose = lidar2camera.inverse() * (pose_ref.inverse() * pose_tar) * lidar2camera;


            auto matching_matrix = get_pillar_matching_gt(ref_pcl, tar_pcl, rel_pose, radius);



            // write binary file
            char out_sub_dir[200];
            sprintf(out_sub_dir, "%02d/pillar_correspondence/delta_%02d/", squence_inx, frm_delta);
            boost::filesystem::create_directories(output_path + out_sub_dir);

            char outfile[100];
            sprintf(outfile, "%06d.mat",  frm_index);
            auto outfile_path = output_path + out_sub_dir + outfile;
            std::ofstream ofs_(outfile_path, std::ios_base::binary);
            if(!ofs_.is_open()) {
                std::cout << "Cannot open file to write!" << std::endl;
                return -1;
            }
            int cols = matching_matrix.cols();
            int rows = matching_matrix.rows();

            ofs_.write(reinterpret_cast<char*>(&rows), sizeof(int));
            ofs_.write(reinterpret_cast<char*>(&cols), sizeof(int));
            auto data_ptr = matching_matrix.data();
            ofs_.write(reinterpret_cast<char*>(data_ptr), sizeof(float) * cols * rows);

            frm_index += 1;
        }
        squence_inx += 1;
    }

}

