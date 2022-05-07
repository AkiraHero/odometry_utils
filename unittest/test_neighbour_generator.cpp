//
// Created by xlju on 22/12/2021.
//

#include <cal_lo_net_feature.hpp>

int main() {
    Eigen::MatrixXf mat;
    mat.resize(3, 5);
    mat.setRandom();
    std::cout << "init matrix\n" << mat << std::endl;
    auto neighbours = get_neighbour_mat(mat);
    std::vector<std::string> names = { "top", "left", "bottom", "right"};

    for(int i = 0; i != neighbours.size(); i++) {
        // top/left/bottom/right
        std::cout << names[i] << "\n" << neighbours[i] << std::endl;
        std::cout << "===============================" << std::endl;
    }
    return 0;
}