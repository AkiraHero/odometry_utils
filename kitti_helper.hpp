#pragma once

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <sstream>
#include <cassert>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>


class KittiHelper {

public:
    static std::shared_ptr<std::vector<float>> read_frm(std::string file_name) {
        std::ifstream ifs(file_name);
        if (! ifs.is_open()) {
            std::cout << "File cannot be open." << std::endl;
        }
        ifs.seekg(0, std::ios::end);
        const size_t num_elements = ifs.tellg() / sizeof(float);
        auto byte2read = num_elements * sizeof(float);
        ifs.seekg(0, std::ios::beg);
        std::shared_ptr<std::vector<float>> buffer =  std::make_shared<std::vector<float>>(num_elements);
        ifs.read(reinterpret_cast<char*>(buffer->data()), byte2read);
        if (byte2read != ifs.gcount()) {
            std::cout << "Reading error!" << std::endl;
        }
        return buffer;
    };

    static pcl::PointCloud<pcl::PointXYZI>::Ptr buffer2cloud(std::shared_ptr<std::vector<float>> buf) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (std::size_t i = 0; i < buf->size(); i += 4) {
            pcl::PointXYZI point;
            point.x = buf->at(i);
            point.y = buf->at(i + 1);
            point.z = buf->at(i + 2);
            point.intensity = buf->at(i + 3);
            laser_cloud->push_back(point);
        }
        return laser_cloud;
    }

    static std::vector<Eigen::Matrix4f> read_pose_file(std::string pose_file) {
        std::vector<Eigen::Matrix4f> result;
        std::ifstream fs_(pose_file);
        if(!fs_.is_open()) {
            std::cout << "open error!" << std::endl;
        }
        char buf[1000];
        char unit_buf[100];
        while(fs_.getline(buf, 1000)) {
            if (fs_.eof()) {
                break;
            }
            std::vector<float> line_nums;
            std::stringstream ss(buf);
            while(ss.getline(unit_buf, 100, 32U)) {
                std::stringstream ss_(unit_buf);
                float num;
                ss_ >> num;
                line_nums.emplace_back(num);
            }
            line_nums.emplace_back(0.0f);
            line_nums.emplace_back(0.0f);
            line_nums.emplace_back(0.0f);
            line_nums.emplace_back(1.0f);
            assert(line_nums.size() == 16);
            Eigen::Matrix4f mat(line_nums.data());
            auto mat_t = mat.transpose();
            result.emplace_back(mat_t);
        }
        return result;
    }


    static std::map<std::string, Eigen::Matrix4f> load_calib_file(std::string calib_file) {
        std::map<std::string, Eigen::Matrix4f> res;

        std::vector<Eigen::Matrix4f> result;
        std::ifstream fs_(calib_file);
        if(!fs_.is_open()) {
            std::cout << "open error!" << std::endl;
        }
        char buf[1000];
        char unit_buf[100];
        while(fs_.getline(buf, 1000)) {
            if (fs_.eof()) {
                break;
            }
            std::vector<float> line_nums;
            auto key = std::string(buf, 2);
            std::stringstream ss(buf + 4);
            while(ss.getline(unit_buf, 100, 32U)) {
                std::stringstream ss_(unit_buf);
                float num;
                ss_ >> num;
                line_nums.emplace_back(num);
            }
            line_nums.emplace_back(0.0f);
            line_nums.emplace_back(0.0f);
            line_nums.emplace_back(0.0f);
            line_nums.emplace_back(1.0f);
            assert(line_nums.size() == 16);
            Eigen::Matrix4f mat(line_nums.data());
            auto mat_t = mat.transpose();
            res[key] = mat_t;
        }
        return res;
    }
};



