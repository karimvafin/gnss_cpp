//
// Created by Карим Вафин on 08.04.2024.
//
#include "gtest/gtest.h"

#include "src/process_sat_data/ProcessSatData.hpp"
#include "src/utils/Parsers.hpp"

#include <iomanip>

class TestProcessData : public ::testing::Test {
protected:
    void SetUp() {
        std::string FILE_PATH = __FILE__;
        std::string dataDirPath = FILE_PATH.substr(0, FILE_PATH.size() - 46) + "/data/";

        const std::string roverFile = dataDirPath + "rover_data.json";
        const auto roverDataOpt = gnss::utils::parseRinex(roverFile);
        ASSERT_TRUE(roverDataOpt.has_value());
        roverData_ = roverDataOpt.value();

        const std::string stationFile = dataDirPath + "station_data.json";
        const auto stationDataOpt = gnss::utils::parseRinex(stationFile);
        ASSERT_TRUE(stationDataOpt.has_value());
        stationData_ = stationDataOpt.value();

        const std::string ephFile = dataDirPath + "satellite_data.json";
        const auto ephDataOpt = gnss::utils::parseSp3(ephFile);
        ASSERT_TRUE(ephDataOpt.has_value());
        ephData_ = ephDataOpt.value();
    }

    std::vector<gnss::RinexData> roverData_;
    std::vector<gnss::RinexData> stationData_;
    std::vector<gnss::Sp3Data> ephData_;
};

TEST(PROCESS_DATA, CALC_ELEV_ANGLE) {
    const Eigen::Vector3d stationPos = {0, 123, 0};
    const Eigen::Vector3d satPos = {std::sqrt(3), 123 + 1, 0};

    const double sinElevAngle = gnss::calcSinElevationAngle(stationPos, satPos);
    ASSERT_NEAR(sinElevAngle, 0.5, 1e-15);
}

TEST_F(TestProcessData, FIND_STATION_DATA) {
    const std::vector<gnss::RinexData> stationData0 = {
        {123.1234, {}}, {123.12545, {}}, {123.12789, {}}, {123.13456, {}}};
    const double timeJd = 123.1254502352;
    const auto resOpt = gnss::findStationData(timeJd, stationData0);
    ASSERT_TRUE(resOpt.has_value());
    ASSERT_TRUE(resOpt.value().first == 1);
    ASSERT_NEAR(resOpt.value().second.timeJD, 123.12545, 1e-15);

    const double epoch0 = roverData_[0].timeJD;
    const auto findStData0 = findStationData(epoch0, stationData_);
    ASSERT_TRUE(findStData0.has_value());
    const unsigned int startIndex = findStData0.value().first;
    ASSERT_TRUE(startIndex == 62);
}

TEST_F(TestProcessData, MATCH_SATELLITES) {
    const auto roverData = roverData_[15];
    const double epoch = roverData.timeJD;
    const auto findStDataOpt = findStationData(epoch, stationData_);
    ASSERT_TRUE(findStDataOpt.has_value());
    const unsigned int startIndex = findStDataOpt.value().first;
    const auto stationData = findStDataOpt.value().second;
    ASSERT_TRUE(startIndex == 77);
    std::unordered_map<std::string, gnss::MatchedSatelliteMeasurements> matched =
        gnss::matchSatelliteMeasurements(roverData, stationData, ephData_);

    for (const auto& x : roverData.satelliteData) {
        std::cout << x.first << " ";
    }
    std::cout << std::endl;
    //
    //    for (const auto& x : stationData.satelliteData) {
    //        std::cout << x.first << " ";
    //    }
    //    std::cout << std::endl;

    for (const auto& x : matched) {
        const auto it1 = roverData.satelliteData.find(x.first);
        ASSERT_TRUE(it1 != roverData.satelliteData.end());
        const auto it2 = stationData.satelliteData.find(x.first);
        ASSERT_TRUE(it2 != stationData.satelliteData.end());
        //        std::cout << x.second.position.x() << " " << x.second.position.y() << " " << x.second.position.z() <<
        //        std::endl; std::cout << x.second.position.norm() << std::endl;
    }
    //    std::cout << std::endl;

//    const std::vector<std::string> satOrder = {"G32", "G27", "G19", "G12", "G25", "G06"};
//    const std::vector<std::string> newSatOrder = gnss::updateSatOrder(satOrder, matched);
//    for (const auto& sat: newSatOrder) std::cout << sat << " ";
//    std::cout << std::endl;
}