//
// Created by Карим Вафин on 24.03.2024.
//
#include "KalmanFilter.hpp"
#include <iostream>
#include "Constants.hpp"
#include "Eigen/Dense"
namespace gnss {

FilterData filterStep(const FilterData& filterData, const Eigen::VectorXd& meas, const Eigen::MatrixXd& F,
                      const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H,
                      const Eigen::VectorXd& measFromX) {
    const unsigned int dim = filterData.x.size();
    const Eigen::MatrixXd P_tmp = F * filterData.P * F.transpose() + Q;
    const Eigen::MatrixXd K = P_tmp * H.transpose() * (H * P_tmp * H.transpose() + R).inverse();
    const Eigen::MatrixXd P_next = (Eigen::MatrixXd::Identity(dim, dim) - K * H) * P_tmp;
    const Eigen::VectorXd x_tmp = F * filterData.x;
    const Eigen::VectorXd residual = (meas - measFromX);
    const Eigen::VectorXd x_next = x_tmp + K * residual;
    //    std::cout << (H * P_tmp * H.transpose() + R) << std::endl << std::endl;
    //    std::cout << K << std::endl << std::endl;
    //    std::cout << P_next << std::endl << std::endl;
    //    std::cout << x_tmp << std::endl << std::endl;
    //    std::cout << x_next << std::endl << std::endl;
    return FilterData{x_next, P_next, residual};
}

Eigen::MatrixXd calcMatrixH(const std::vector<Eigen::Vector3d>& satEphemeris, const Eigen::Vector3d& stationPosition,
                            const unsigned int refSatIndex) {
    assert(refSatIndex < satEphemeris.size());
    const unsigned int measDimension = 2 * (satEphemeris.size() - 1);
    const unsigned int stateDimension = 3 + satEphemeris.size();
    Eigen::MatrixXd res{measDimension, stateDimension};
    for (unsigned int i = 0; i < measDimension; ++i) {
        for (unsigned int j = 0; j < stateDimension; ++j) {
            res(i, j) = 0;
        }
    }
    const Eigen::Vector3d& refSatEph = satEphemeris[refSatIndex];
    const Eigen::Vector3d refSatRelPosNorm = (refSatEph - stationPosition).normalized();
    for (unsigned int i = 0; i < measDimension / 2; ++i) {
        const unsigned int satIndex = i < refSatIndex ? i : i + 1;
        const Eigen::Vector3d satRelPosNorm = (satEphemeris[satIndex] - stationPosition).normalized();
        const Eigen::Vector3d diff = satRelPosNorm - refSatRelPosNorm;
        res(i, 0) = -diff.x();
        res(i, 1) = -diff.y();
        res(i, 2) = -diff.z();
        res(measDimension / 2 + i, 0) = res(i, 0);
        res(measDimension / 2 + i, 1) = res(i, 1);
        res(measDimension / 2 + i, 2) = res(i, 2);
        res(measDimension / 2 + i, 3 + satIndex) = 1;
        res(measDimension / 2 + i, 3 + refSatIndex) = -1;
    }
    return res;
}

}  // namespace gnss