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
    std::string outputDirPath = FILE_PATH.substr(0, FILE_PATH.size() - 40) + "/output/out.csv";
    FILE* f;
    f = fopen(outputDirPath.c_str(), "w");
    fprintf(f, "num,epoch,resx,resy,resz,x,y,z\n");

    const std::string roverFile = dataDirPath + "rover_data_1.json";
    const auto roverDataOpt = utils::parseRinex(roverFile);
    ASSERT_TRUE(roverDataOpt.has_value());
    const std::vector<RinexData>& roverData = roverDataOpt.value();

    const std::string stationFile = dataDirPath + "station_data_1.json";
    const auto stationDataOpt = utils::parseRinex(stationFile);
    ASSERT_TRUE(stationDataOpt.has_value());
    const std::vector<RinexData>& stationData = stationDataOpt.value();

    const std::string ephFile = dataDirPath + "satellite_data_1.json";
    const auto ephDataOpt = utils::parseSp3(ephFile);
    ASSERT_TRUE(ephDataOpt.has_value());
    const std::vector<Sp3Data>& ephData = ephDataOpt.value();

    const Eigen::Vector3d stationPos = {2822740.319, 2204989.12, 5259903.067};
    Eigen::VectorXd x{{0, 0, 0}};
    Eigen::MatrixXd P{
        {Constants::sigmaPosition, 0, 0}, {0, Constants::sigmaPosition, 0}, {0, 0, Constants::sigmaPosition}};
    FilterData filterData{x, P, {}};
    std::vector<std::string> satOrder;
    for (int i = 0; i < roverData.size(); ++i) {
        // получение измерений со станции
        const RinexData& roverMeas = roverData[i];
        const auto stationMeasOpt = findStationData(roverMeas.timeJD, stationData);
        std::cout << stationMeasOpt.value().first << std::endl;
        if (!stationMeasOpt.has_value()) {
            continue;
        }
        //const auto& [startIndex, stationMeas] = stationMeasOpt.value();

        // нахождение пересечения спутников ровера и станции, вычисление эфемерид спутников
        const std::unordered_map<std::string, MatchedSatelliteMeasurements> matchedSats =
            matchSatelliteMeasurements(roverMeas, stationMeasOpt.value().second, ephData);

        // согласование порядкового номера спутников с предыдущего шага
        const std::vector<std::string> newSatOrder = updateSatOrder(satOrder, matchedSats);

        // определение опорного спутника
        //const unsigned int refSatIndex = calcRefSatIndex(newSatOrder, matchedSats, stationPos);
        const unsigned int refSatIndex = 0;

        // количество спутников
        const unsigned int currSatNum = newSatOrder.size();

        // расчет первых разностей
        std::vector<double> firstDiff = calcFirstDiff(newSatOrder, matchedSats);

        // расчет вторых разностей (вектор измерений z)
        Eigen::VectorXd secondDiff = calcSecondDiff(firstDiff, refSatIndex);
        const unsigned int measDim = secondDiff.size();

        // начальное приближение
        filterData = recalcFilterData(filterData, satOrder, newSatOrder, firstDiff);
        const unsigned int dim = filterData.x.size();

        Eigen::MatrixXd H = calcMatrixH(getSatEphemeris(newSatOrder, matchedSats), stationPos, refSatIndex);
        Eigen::MatrixXd Q(dim, dim);
        Q.setZero();
        Q(0, 0) = Constants::sigmaPosition;
        Q(1, 1) = Constants::sigmaPosition;
        Q(2, 2) = Constants::sigmaPosition;
        Eigen::MatrixXd R(measDim, measDim);
        R.setZero();
        constexpr double measRelError = 1e-2;
        for (unsigned int j = 0; j < measDim; ++j) R(j, j) = measRelError * secondDiff[j];

        //        std::cout << filterData.x << std::endl << std::endl;
        //        std::cout << filterData.P << std::endl << std::endl;
        //        std::cout << secondDiff << std::endl << std::endl;
        //        std::cout << Q << std::endl << std::endl;
        //        std::cout << R << std::endl << std::endl;
        //        std::cout << H << std::endl << std::endl;
        const Eigen::VectorXd measFromX =
            calcMeasurement(filterData.x, newSatOrder, matchedSats, refSatIndex, stationPos);
        filterData = filterStep(filterData, secondDiff, Eigen::MatrixXd::Identity(dim, dim), Q, R, H, measFromX);
        fprintf(f, "%d,%f,%f,%f,%f,%f,%f,%f\n", i, roverMeas.timeJD - 2.46032e+06, filterData.residual(0),
                filterData.residual(1), filterData.residual(2), filterData.x(0), filterData.x(1), filterData.x(2));
        std::cout << "Epoch: " << roverMeas.timeJD - 2.46032e+06 << " Dimension: " << dim
                  << "  Result: " << filterData.x(0) << " " << filterData.x(1) << " " << filterData.x(2)
                  << " Ref sat: " << refSatIndex << std::endl;
        satOrder = newSatOrder;
    }

    fclose(f);
}