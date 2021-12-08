#pragma once

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

};



