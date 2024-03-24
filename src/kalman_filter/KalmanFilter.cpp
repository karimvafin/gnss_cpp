//
// Created by Карим Вафин on 24.03.2024.
//
#include "KalmanFilter.hpp"

namespace gnss {

FilterData filterStep(const FilterData& filterData, const Eigen::VectorXd meas, const Eigen::MatrixXd& F,
                      const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H) {
    const unsigned int dim = filterData.x.size();
    const Eigen::MatrixXd P_tmp = F * filterData.P * F.transpose() + Q;
    const Eigen::MatrixXd K = P_tmp * H.transpose() * (H * P_tmp * H.transpose() + R).inverse();
    const Eigen::MatrixXd P_next = (Eigen::MatrixXd::Identity(dim, dim) - K * H) * P_tmp;
    const Eigen::VectorXd x_tmp = F * filterData.x;
    const Eigen::VectorXd x_next = x_tmp + K * (meas - H * x_tmp);
    return FilterData{x_next, P_next};
}

}  // namespace gnss