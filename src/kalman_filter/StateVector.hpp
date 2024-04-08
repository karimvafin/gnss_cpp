//
// Created by Карим Вафин on 27.03.2024.
//

#ifndef GNSS_CPP_STATEVECTOR_HPP
#define GNSS_CPP_STATEVECTOR_HPP

#include "third_party/eigen/Eigen/Core"

namespace gnss {

Eigen::Vector3d getPosition(const Eigen::VectorXd& state);

}  // namespace gnss
#endif  // GNSS_CPP_STATEVECTOR_HPP
