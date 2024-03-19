//
// Created by Карим Вафин on 17.03.2024.
//

#ifndef GNSS_CPP_LAGRANGEINTERPOLATION_HPP
#define GNSS_CPP_LAGRANGEINTERPOLATION_HPP

#include <vector>
#include "third_party/eigen/Eigen/Core"

namespace gnss {

double lagrangeInterpolation(double x0, const std::vector<double> &x, const std::vector<double> &y);

Eigen::Vector3d lagrangeInterpolation(double x0, const std::vector<double> &x, const std::vector<Eigen::Vector3d> &y);

}  // namespace gnss
#endif  // GNSS_CPP_LAGRANGEINTERPOLATION_HPP
