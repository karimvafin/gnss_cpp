//
// Created by Карим Вафин on 04.04.2024.
//
#include "src/kalman_filter/KalmanFilter.hpp"
#include "gtest/gtest.h"

TEST(KALMAN_FILTER, H_MATRIX) {
    const std::vector<Eigen::Vector3d> satEphemeris = {Eigen::Vector3d{1, 1, 0}, Eigen::Vector3d{2, 1, 0},
                                                       Eigen::Vector3d{3, 1, 0}, Eigen::Vector3d{4, 1, 0}};
    const unsigned int refSatIndex = 0;
    const Eigen::Vector3d stationPos = {0, 0, 0};
    const Eigen::MatrixXd H = gnss::calcMatrixH(satEphemeris, stationPos, refSatIndex);
    std::cout << H << std::endl;
}