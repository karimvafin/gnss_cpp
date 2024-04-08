//
// Created by Карим Вафин on 04.04.2024.
//
#include "src/Constants.hpp"
#include "src/ephemeris/SatelliteEphemeris.hpp"
#include "src/kalman_filter/KalmanFilter.hpp"
#include "src/process_sat_data/ProcessSatData.hpp"
#include "src/types/InputTypes.hpp"
#include "src/utils/Parsers.hpp"

#include "gtest/gtest.h"

using namespace gnss;

TEST(FILTRATION, TEST1) {
    std::string FILE_PATH = __FILE__;
    std::string dataDirPath = FILE_PATH.substr(0, FILE_PATH.size() - 40) + "/data/";

    const std::string roverFile = dataDirPath + "rover_data.json";
    const auto roverDataOpt = utils::parseRinex(roverFile);
    ASSERT_TRUE(roverDataOpt.has_value());
    const std::vector<RinexData>& roverData = roverDataOpt.value();

    const std::string stationFile = dataDirPath + "station_data.json";
    const auto stationDataOpt = utils::parseRinex(stationFile);
    ASSERT_TRUE(stationDataOpt.has_value());
    const std::vector<RinexData>& stationData = stationDataOpt.value();

    const std::string ephFile = dataDirPath + "satellite_data.json";
    const auto ephDataOpt = utils::parseSp3(ephFile);
    ASSERT_TRUE(ephDataOpt.has_value());
    const std::vector<Sp3Data>& ephData = ephDataOpt.value();

    //    const int size = (stationData.size() > roverData.size() ? roverData.size() : stationData.size());
    //    std::cout << std::setprecision(15);
    //    for (int i = 0; i < size; ++i) {
    //        std::cout << stationData[i].timeJD << " " << roverData[i].timeJD << std::endl;
    //    }

    const Eigen::Vector3d stationPos = {0, 0, 0};
    Eigen::VectorXd x(3);
    x(0) = 0; x(1) = 0; x(2) = 0;
    std::vector<std::string> satOrder;
    for (int i = 0; i < roverData.size(); ++i) {
        // получение измерений со станции
        const RinexData& roverMeas = roverData[i];
        const auto stationMeasOpt = findStationData(roverMeas.timeJD, stationData);
        if (!stationMeasOpt.has_value()) {
            continue;
        }
        const RinexData& stationMeas = stationMeasOpt.value();

        // нахождение пересечения спутников ровера и станции, вычисление эфемерид спутников
        const std::unordered_map<std::string, MatchedSatelliteMeasurements> matchedSats =
            matchSatelliteMeasurements(roverMeas, stationMeas, ephData);

        // согласование порядкового номера спутников с предыдущего шага
        const std::vector<std::string> newSatOrder = updateSatOrder(satOrder, matchedSats);

        // определение опорного спутника
        const unsigned int refSatIndex = calcRefSatIndex(newSatOrder, matchedSats, stationPos);

        // количество спутников
        const unsigned int currSatNum = newSatOrder.size();

        // расчет первых разностей
        std::vector<double> firstDiff = calcFirstDiff(newSatOrder, matchedSats);

        // расчет вторых разностей (вектор измерений z)
        Eigen::VectorXd secondDiff = calcSecondDiff(firstDiff, refSatIndex);

        // начальное приближение

    }
}