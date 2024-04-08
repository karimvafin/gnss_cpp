//
// Created by Карим Вафин on 24.03.2024.
//

#ifndef GNSS_CPP_KALMANFILTER_HPP
#define GNSS_CPP_KALMANFILTER_HPP

#include "third_party/eigen/Eigen/Core"
#include "src/types/InputTypes.hpp"

namespace gnss {

struct FilterData {
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
};

FilterData filterStep(const FilterData& filterData, const Eigen::VectorXd &meas, const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q,
                      const Eigen::MatrixXd& R, const Eigen::MatrixXd& H);

Eigen::MatrixXd calcMatrixH(const std::vector<Eigen::Vector3d>& satEphemeris, const Eigen::Vector3d& stationPosition,
                            unsigned int refSatIndex);

}  // namespace gnss
#endif  // GNSS_CPP_KALMANFILTER_HPP
