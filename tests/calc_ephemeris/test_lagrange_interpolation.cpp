//
// Created by Карим Вафин on 18.03.2024.
//

#include "gtest/gtest.h"
#include "src/LagrangeInterpolation.hpp"

TEST(LAGRANGE_INTERPOLATION, TEST4) {
    std::vector<double> x = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
    std::vector<double> y = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8};
    const double res = gnss::lagrangeInterpolation(0.55, x, y);
    ASSERT_TRUE(false);
    ASSERT_DOUBLE_EQ(res, 1.3);
}