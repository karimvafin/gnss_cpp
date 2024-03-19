//
// Created by Карим Вафин on 18.03.2024.
//

#include "gtest/gtest.h"
#include "src/utils/LagrangeInterpolation.hpp"

TEST(LAGRANGE_INTERPOLATION, TEST1) {
    std::vector<double> x = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    std::vector<double> y = {0.0, 0.01, 0.04, 0.09, 0.16, 0.25, 0.36, 0.49, 0.64, 0.81};
    const double res = gnss::lagrangeInterpolation(0.55, x, y);
    ASSERT_DOUBLE_EQ(res, 0.3025);
}