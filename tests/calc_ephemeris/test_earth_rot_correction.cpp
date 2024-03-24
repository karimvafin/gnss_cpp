//
// Created by Карим Вафин on 20.03.2024.
//
#include "src/ephemeris/SatelliteEphemeris.hpp"

#include "gtest/gtest.h"

TEST(EARTH_ROT_CORR, FULL_ANGLE) {
    Eigen::Vector3d vec{0, 1, 0};
    double time = 86400;
    Eigen::Vector3d res = gnss::correctEarthRotation(vec, time);
    double tol = 1e-15;
    ASSERT_NEAR(res.x(), vec.x(), tol);
    ASSERT_NEAR(res.y(), vec.y(), tol);
    ASSERT_NEAR(res.z(), vec.z(), tol);
}