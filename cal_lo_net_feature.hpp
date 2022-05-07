//
// Created by xlju on 19/12/2021.
//
#pragma once

#include <map>
#include <cassert>
#include <algorithm>
#include "kitti_helper.hpp"
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <chrono>
// generate H * W * C=6(range, intensity, n_x, n_y, n_z, x, y, z)


int H = 64;
int W = 1800;
float max_vertical_angle = 4.09103f / 180.0f * M_PI;
float min_vertical_angle = -23.0345f / 180.0f * M_PI;
float max_horizontal_angle = 3.14158f;
float min_horizontal_angle = -3.14154f;
float delta_alpha = (max_horizontal_angle - min_horizontal_angle) / W;
float delta_beta = (max_vertical_angle - min_vertical_angle) / (H - 1);

void print_pcl_angle_range(pcl::PointCloud<pcl::PointXYZI>::Ptr p_pcl,
                           std::pair<float, float> *vertical_range = nullptr,
                           std::pair<float, float> *horizontal_range = nullptr
                           ) {
    // calculate point angle distribution
    std::vector<float> horizontal_angles;
    std::vector<float> vertical_angles;
    std::vector<float> ranges;
    for(auto const& c: p_pcl->points) {
        auto range = sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
        ranges.emplace_back(range);
        horizontal_angles.emplace_back(atan2(c.y, c.x));
        auto vertical_angle = atan2(c.z, range);
        vertical_angles.emplace_back(vertical_angle);
    }
    if (vertical_range != nullptr) {
        (*vertical_range).first = *(std::min_element(vertical_angles.begin(), vertical_angles.end()));
        (*vertical_range).second = *(std::max_element(vertical_angles.begin(), vertical_angles.end()));
    }
    if (horizontal_range != nullptr) {
        (*horizontal_range).first = *(std::min_element(horizontal_angles.begin(), horizontal_angles.end()));
        (*horizontal_range).second = *(std::max_element(horizontal_angles.begin(), horizontal_angles.end()));
    }
    std::cout << "horizontal_angles:" << *(std::max_element(horizontal_angles.begin(), horizontal_angles.end())) << "~" << *(std::min_element(horizontal_angles.begin(), horizontal_angles.end())) << std::endl;
    std::cout << "vertical_angles:" << *(std::max_element(vertical_angles.begin(), vertical_angles.end())) << "~" << *(std::min_element(vertical_angles.begin(), vertical_angles.end())) << std::endl;
}

template<class PointType>
std::pair<int, int> get_img_inx(PointType c) {
    auto range = sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
    auto horizontal_angle = atan2(c.y, c.x);
    auto vertical_angle = asin(c.z / range);
    int col = (horizontal_angle - min_horizontal_angle) / delta_alpha;
    int row = (vertical_angle - min_vertical_angle) / delta_beta;
    return std::make_pair(row, col);
}


// return top/left/bottom/right
std::vector<Eigen::MatrixXf> get_neighbour_mat(Eigen::MatrixXf const&x) {
    Eigen::MatrixXf x_top, x_bottom, x_left, x_right;
    auto rows = x.rows();
    auto cols = x.cols();
    x_top.resize(rows, cols);
    x_bottom.resize(rows, cols);
    x_left.resize(rows, cols);
    x_right.resize(rows, cols);

    x_top.block(1, 0, x.rows() - 1, x.cols()) = x.block(0, 0, x.rows() - 1, x.cols());
    x_top.row(0) = x.row(0);

    x_bottom.block(0, 0, x.rows() - 1, x.cols()) = x.block(1, 0, x.rows() - 1, x.cols());
    x_bottom.row(rows - 1) = x.row(rows - 1);

    x_left.block(0, 1, x.rows() , x.cols() - 1) = x.block(0, 0, x.rows(), x.cols() - 1);
    x_left.col(0) = x.col(0);

    x_right.block(0, 0, x.rows() , x.cols() - 1) = x.block(0, 1, x.rows(), x.cols() - 1);
    x_right.col(cols - 1) = x.col(cols - 1);

    return std::vector<Eigen::MatrixXf>{
        x_top, x_left, x_bottom, x_right
    };
}

void cal_normal_lonet(Eigen::MatrixXf const&x, Eigen::MatrixXf const&y, Eigen::MatrixXf const&z, Eigen::MatrixXf const&r,
                     Eigen::MatrixXf &n_x, Eigen::MatrixXf &n_y, Eigen::MatrixXf &n_z) {

    // counterclockwise
    std::vector<std::pair<int, int>> neighbour_inx_pair = {
            {3, 0},
            {0, 1},
            {1, 2},
            {2, 3}
    };
//    auto st1 = std::chrono::high_resolution_clock().now();
    auto neighbour_x = get_neighbour_mat(x);
    auto neighbour_y = get_neighbour_mat(y);
    auto neighbour_z = get_neighbour_mat(z);
    auto neighbour_r = get_neighbour_mat(r);
//    auto st2 = std::chrono::high_resolution_clock().now();
//    std::cout << "get nei cost:" << (st2 - st1).count() << std::endl;

    auto get_neighbour_diff = [](const std::vector<Eigen::MatrixXf> &a,  const Eigen::MatrixXf &b) {
        std::vector<Eigen::ArrayXXf> res;
        for(const auto & i : a) {
            res.emplace_back((i - b).array());
        }
        return res;
    };

    auto neighbour_diff_x = get_neighbour_diff(neighbour_x, x);
    auto neighbour_diff_y = get_neighbour_diff(neighbour_y, y);
    auto neighbour_diff_z = get_neighbour_diff(neighbour_z, z);
    auto neighbour_weight_r = get_neighbour_diff(neighbour_r, r);
    for(auto &c: neighbour_weight_r){
        auto x = (-0.2 * c.abs()).exp();
        c = std::move(x);
    }
    Eigen::ArrayXXf norm_x, norm_y, norm_z;
    auto rows = x.rows();
    auto cols = x.cols();
    norm_x.setZero();
    norm_y.setZero();
    norm_z.setOnes();
    norm_x.resize(rows, cols);
    norm_y.resize(rows, cols);
    norm_z.resize(rows, cols);


//    auto st3 = std::chrono::high_resolution_clock().now();
//    std::cout << "get get_neighbour_diff cost:" << (st3 - st2).count() << std::endl;

    for(auto const &inx_pair: neighbour_inx_pair) {
        int const &k = inx_pair.first;
        int const &j = inx_pair.second;
        auto const &w_k = neighbour_weight_r[k];
        auto const &w_j = neighbour_weight_r[j];
        auto w = w_k * w_j;
        norm_x += w * (neighbour_diff_y[k] * neighbour_diff_z[j] - neighbour_diff_z[k] * neighbour_diff_y[j]);
        norm_y += w * (neighbour_diff_z[k] * neighbour_diff_x[j] - neighbour_diff_x[k] * neighbour_diff_z[j]);
        norm_z += w * (neighbour_diff_x[k] * neighbour_diff_y[j] - neighbour_diff_y[k] * neighbour_diff_x[j]);
    }
//    auto mod = Eigen::sqrt(norm_x * norm_x + norm_y * norm_y + norm_z * norm_z);
//    auto ini_n_x = (norm_x / mod);
//    auto ini_n_y = (norm_y / mod);
//    auto ini_n_z = (norm_z / mod);


//    auto st4 = std::chrono::high_resolution_clock().now();
//    std::cout << "get norm cost:" << (st4 - st3).count() << std::endl;

    // moving average filter
    auto neighbour_nx = get_neighbour_mat(norm_x);
    auto neighbour_ny = get_neighbour_mat(norm_y);
    auto neighbour_nz = get_neighbour_mat(norm_z);
    auto n_x_ = (neighbour_nx[0] + neighbour_nx[1] + neighbour_nx[2] + neighbour_nx[3]).array();
    auto n_y_  = (neighbour_ny[0] + neighbour_ny[1] + neighbour_ny[2] + neighbour_ny[3]).array();
    auto n_z_  = (neighbour_nz[0] + neighbour_nz[1] + neighbour_nz[2] + neighbour_nz[3]).array();
    auto mod_ = Eigen::sqrt(n_x_ * n_x_ + n_y_ * n_y_ + n_z_ * n_z_) + 1e-12; // avoid dividing zero
    n_x = (n_x_ / mod_).matrix();
    n_y = (n_y_ / mod_).matrix();
    n_z = (n_z_ / mod_).matrix();

//
//    auto st5 = std::chrono::high_resolution_clock().now();
//    std::cout << " norm filter cost:" << (st5 - st4).count() << std::endl;
}



std::map<std::string, Eigen::MatrixXf> generate_range_img(pcl::PointCloud<pcl::PointXYZI>::Ptr p_pcl, float max_range=120.0) {
//    auto st1 = std::chrono::high_resolution_clock().now();
    std::map<std::string, Eigen::MatrixXf> res;
    Eigen::MatrixXf mat_r; mat_r.resize(H, W); mat_r.setZero();
    Eigen::MatrixXf mat_x; mat_x.resize(H, W); mat_x.setZero();
    Eigen::MatrixXf mat_y; mat_y.resize(H, W); mat_y.setZero();
    Eigen::MatrixXf mat_z; mat_z.resize(H, W); mat_z.setZero();
    Eigen::MatrixXf mat_i; mat_i.resize(H, W); mat_i.setZero();
    Eigen::MatrixXf mat_nx; mat_nx.resize(H, W); mat_nx.setZero();
    Eigen::MatrixXf mat_ny; mat_ny.resize(H, W); mat_ny.setZero();
    Eigen::MatrixXf mat_nz; mat_nz.resize(H, W); mat_nz.setZero();
    Eigen::MatrixXi mat_inx; mat_inx.resize(H, W); mat_inx.setZero();
//    auto st2 = std::chrono::high_resolution_clock().now();

// if want to cal norm using pcl library
//    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
//    ne.setInputCloud(p_pcl);
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
//    ne.setSearchMethod(tree);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//    ne.setRadiusSearch (0.5);
//    ne.compute (*cloud_normals);


    for(int inx = 0; inx != p_pcl->points.size(); inx++) {
        auto c = p_pcl->points[inx];
        auto coordinate = get_img_inx(c);
        if(coordinate.first < 0 || coordinate.first >= H)
            continue;
        if(coordinate.second < 0 || coordinate.second >= W)
            continue;
        mat_inx(coordinate.first, coordinate.second) = 1;

        float r = sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
        if (r > max_range) {
            r = 0.0f;
            c.x = c.y = c.z = c.intensity = 0.0f;
        }
        mat_x(coordinate.first, coordinate.second) = c.x;
        mat_y(coordinate.first, coordinate.second) = c.y;
        mat_z(coordinate.first, coordinate.second) = c.z;
        mat_i(coordinate.first, coordinate.second) = c.intensity;
        mat_r(coordinate.first, coordinate.second) = r;
    }
//    auto st3 = std::chrono::high_resolution_clock().now();

    // interpolate
    std::vector<std::pair<int, int>> neighbour_inx_pair = {
            {0, 1},
            {0, -1},
            {1, 0},
            {-1, 0},
    };
    for(int i = 0; i != mat_inx.rows(); i++) {
        for(int j = 0; j != mat_inx.cols(); j++) {
            int valid_pixel_num = 0;
            if(mat_inx(i, j) == 0) {
                for(int k = 0; k != 4; k++) {
                    int neibour_x = i + neighbour_inx_pair[k].first;
                    int neibour_y = j + neighbour_inx_pair[k].second;
                    if(neibour_x >= 0 && neibour_x < mat_inx.rows() &&
                    neibour_y >= 0 && neibour_y < mat_inx.cols() &&
                    mat_inx(neibour_x, neibour_y) != 0) {
                        valid_pixel_num += 1;
                        mat_x(i, j) += mat_x(neibour_x, neibour_y);
                        mat_y(i, j) += mat_y(neibour_x, neibour_y);
                        mat_z(i, j) += mat_z(neibour_x, neibour_y);
                        mat_i(i, j) += mat_i(neibour_x, neibour_y);
                    }
                }
                if(valid_pixel_num) {
                    mat_x(i, j) /= valid_pixel_num;
                    mat_y(i, j) /= valid_pixel_num;
                    mat_z(i, j) /= valid_pixel_num;
                    mat_i(i, j) /= valid_pixel_num;
                    mat_r(i, j) = sqrt(mat_x(i, j) * mat_x(i, j) + mat_y(i, j) * mat_y(i, j) + mat_z(i, j) * mat_z(i, j));
                    mat_inx(i, j) = 1;
                }
            }
        }
    }
//    auto st4 = std::chrono::high_resolution_clock().now();


    // cal normal according to the paper
    cal_normal_lonet(mat_x, mat_y, mat_z, mat_r, mat_nx, mat_ny, mat_nz);

//    auto st5 = std::chrono::high_resolution_clock().now();
//    std::cout << "cost1: " << (st2 - st1).count() << std::endl;
//    std::cout << "cost2: " << (st3 - st2).count() << std::endl;
//    std::cout << "cost3: " << (st4 - st3).count() << std::endl;
//    std::cout << "cost4: " << (st5 - st4).count() << std::endl;


    res["mat_x"] = mat_x;
    res["mat_y"] = mat_y;
    res["mat_z"] = mat_z;
    res["mat_i"] = mat_i;
    res["mat_r"] = mat_r;
    res["mat_nx"] = mat_nx;
    res["mat_ny"] = mat_ny;
    res["mat_nz"] = mat_nz;
    return res;
}
