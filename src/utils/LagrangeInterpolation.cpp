//
// Created by Карим Вафин on 17.03.2024.
//
#include "LagrangeInterpolation.hpp"

#include <assert.h>

namespace gnss {

double lagrangeInterpolation(double x0, const std::vector<double> &x, const std::vector<double> &y) {
    assert(x.size() == y.size());
    const int n = x.size();
    double res = 0.;
    for (int i = 0; i < n; ++i) {
        double accum = 1.;
        for (int j = 0; j < i; ++j) {
            accum *= (x0 - x[j]) / (x[i] - x[j]);
        }
        for (int j = i + 1; j < n; ++j) {
            accum *= (x0 - x[j]) / (x[i] - x[j]);
        }
        res += y[i] * accum;
    }
    return res;
}

Eigen::Vector3d lagrangeInterpolation(double x0, const std::vector<double> &x, const std::vector<Eigen::Vector3d> &y) {
    assert(x.size() == y.size());
    const int n = x.size();
    Eigen::Vector3d res = Eigen::Vector3d::Zero();
    for (int i = 0; i < n; ++i) {
        double accum = 1.;
        for (int j = 0; j < i; ++j) {
            accum *= (x0 - x[j]) / (x[i] - x[j]);
        }
        for (int j = i + 1; j < n; ++j) {
            accum *= (x0 - x[j]) / (x[i] - x[j]);
        }
        res += y[i] * accum;
    }
    return res;
}

}  // namespace gnss