//
// Created by Карим Вафин on 08.04.2024.
//
#include "gtest/gtest.h"

#include "src/process_sat_data/ProcessSatData.hpp"

TEST(PROCESS_DATA, CALC_ELEV_ANGLE) {
    const Eigen::Vector3d stationPos = {0, 123, 0};
    const Eigen::Vector3d satPos = {std::sqrt(3), 123 + 1, 0};

    const double sinElevAngle = gnss::calcSinElevationAngle(stationPos, satPos);
    ASSERT_NEAR(sinElevAngle, 0.5, 1e-15);
}

TEST(PROCESS_DATA, FIND_STATION_DATA) {
    const std::vector<gnss::RinexData> stationData = {
        {123.1234, {}}, {123.12545, {}}, {123.12789, {}}, {123.13456, {}}};
    const double timeJd = 123.1254502352;
    const auto resOpt = gnss::findStationData(timeJd, stationData);
    ASSERT_TRUE(resOpt.has_value());
    std::cout << resOpt.value().timeJD << std::endl;
    ASSERT_NEAR(resOpt.value().timeJD, 123.12545, 1e-15);
}