//
// Created by Карим Вафин on 06.04.2024.
//

#ifndef GNSS_CPP_PROCESSSATDATA_HPP
#define GNSS_CPP_PROCESSSATDATA_HPP

#include "Eigen/Core"
#include "src/kalman_filter/KalmanFilter.hpp"
#include "src/types/InputTypes.hpp"

#include <optional>

namespace gnss {

double calcSinElevationAngle(const Eigen::Vector3d& stationPos, const Eigen::Vector3d& satPos);

std::optional<RinexData> findStationData(double timeJd, const std::vector<RinexData>& stationData,
                                         unsigned int startIndex = 0);

struct MatchedSatelliteMeasurements {
    double timeJd;
    PhaseCodeMeas roverMeas;
    PhaseCodeMeas stationMeas;
    Eigen::Vector3d position;
};

std::unordered_map<std::string, MatchedSatelliteMeasurements> matchSatelliteMeasurements(
    const RinexData& roverData, const RinexData& stationData, const std::vector<Sp3Data>& ephData);

std::vector<std::string> updateSatOrder(
    const std::vector<std::string>& satOrder,
    const std::unordered_map<std::string, MatchedSatelliteMeasurements>& matchedSats);

unsigned int calcRefSatIndex(const std::vector<std::string>& satOrder,
                             const std::unordered_map<std::string, MatchedSatelliteMeasurements>& matchedSats,
                             const Eigen::Vector3d& stationPos);

std::vector<double> calcFirstDiff(const std::vector<std::string>& satOrder,
                                  const std::unordered_map<std::string, MatchedSatelliteMeasurements>& matchedSats);

Eigen::VectorXd calcSecondDiff(const std::vector<double>& firstDiff, unsigned int refSatIndex);

FilterData recalcFilterData(const FilterData& filterData, const std::vector<std::string>& prevSatOrder,
                            const std::vector<std::string>& satOrder, const std::vector<double>& singleDiff);

}  // namespace gnss

#endif  // GNSS_CPP_PROCESSSATDATA_HPP
