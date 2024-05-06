//
// Created by Карим Вафин on 06.04.2024.
//
#include "ProcessSatData.hpp"
#include <unordered_set>
#include "src/Constants.hpp"
#include "src/ephemeris/SatelliteEphemeris.hpp"
#include <iostream>

namespace gnss {

double calcSinElevationAngle(const Eigen::Vector3d& stationPos, const Eigen::Vector3d& satPos) {
    const Eigen::Vector3d n1 = stationPos.normalized();
    const Eigen::Vector3d n2 = (satPos - stationPos).normalized();
    return n1.dot(n2);
}

std::optional<std::pair<unsigned int, RinexData>> findStationData(double timeJd,
                                                                  const std::vector<RinexData>& stationData,
                                                                  unsigned int startIndex) {
    double currTime = stationData[startIndex].timeJD;
    constexpr double epsilon = 1. / 86400;
    constexpr double threshold = 10 * epsilon;
    while (std::abs(currTime - timeJd) > epsilon && (currTime - timeJd) < threshold) {
        currTime = stationData[++startIndex].timeJD;
    }
    if (std::abs(currTime - timeJd) <= epsilon) {
        return std::pair{startIndex, stationData[startIndex]};
    }
    return std::nullopt;
}

std::unordered_map<std::string, MatchedSatelliteMeasurements> matchSatelliteMeasurements(
    const RinexData& roverData, const RinexData& stationData, const std::vector<Sp3Data>& ephData) {
    const double epoch = roverData.timeJD;
    const auto itStationEnd = stationData.satelliteData.end();
    std::unordered_map<std::string, MatchedSatelliteMeasurements> res;
    for (auto it = roverData.satelliteData.begin(), ite = roverData.satelliteData.end(); it != ite; ++it) {
        const std::string& satName = it->first;
        const auto itStation = stationData.satelliteData.find(satName);
        if (itStation == itStationEnd) {
            continue;
        }
        const double deltaTimeSeconds = it->second.C1W / Constants::lightVelocity;
        const double ephTime = epoch - deltaTimeSeconds / Constants::secondsInDay;
        const auto ephSatOpt = interpolateSatelliteEphemeris(satName, ephTime, ephData);
        if (!ephSatOpt.has_value()) {
            continue;
        }
        const Eigen::Vector3d satEph = correctEarthRotation(ephSatOpt.value(), deltaTimeSeconds);
        res.insert({satName, MatchedSatelliteMeasurements{epoch, it->second, itStation->second, satEph}});
    }
    return res;
}

std::vector<std::string> updateSatOrder(
    const std::vector<std::string>& satOrder,
    const std::unordered_map<std::string, MatchedSatelliteMeasurements>& matchedSats) {
    const unsigned int satNum = satOrder.size();
    std::vector<std::string> newSats;
    std::vector<std::string> res;
    for (auto it = matchedSats.begin(), ite = matchedSats.end(); it != ite; ++it) {
        unsigned int i;
        for (i = 0; (i < satNum) && (it->first != satOrder[i]); ++i)
            ;
        if (i == satNum)
            newSats.push_back(it->first);
        else
            res.push_back(it->first);
    }
    for (const auto& elem : newSats) {
        res.push_back(elem);
    }
    return res;
}

unsigned int calcRefSatIndex(const std::vector<std::string>& satOrder,
                             const std::unordered_map<std::string, MatchedSatelliteMeasurements>& matchedSats,
                             const Eigen::Vector3d& stationPos) {
    unsigned int res = 0;
    double maxSinElevationAngle = 0;
    for (unsigned int i = 0; i < satOrder.size(); ++i) {
        double sinElevAngle = calcSinElevationAngle(stationPos, matchedSats.at(satOrder[i]).position);
        if (sinElevAngle > maxSinElevationAngle) {
            maxSinElevationAngle = sinElevAngle;
            res = i;
        }
    }
    return res;
}

std::vector<double> calcFirstDiff(const std::vector<std::string>& satOrder,
                                  const std::unordered_map<std::string, MatchedSatelliteMeasurements>& matchedSats) {
    const unsigned int satNum = satOrder.size();
    std::vector<double> firstDiff(2 * satNum);
    // сначала все кодовые, затем фазовые измерения
    for (unsigned int i = 0; i < satNum; ++i) {
        firstDiff[i] =
            (matchedSats.at(satOrder[i]).roverMeas.C1W - matchedSats.at(satOrder[i]).stationMeas.C1W);  // в метрах
        firstDiff[satNum + i] =
            (matchedSats.at(satOrder[i]).roverMeas.L1W - matchedSats.at(satOrder[i]).stationMeas.L1W) *
            Constants::waveL1;  // в метрах
    }
    return firstDiff;
}

Eigen::VectorXd calcSecondDiff(const std::vector<double>& firstDiff, unsigned int refSatIndex) {
    const unsigned int satNum = firstDiff.size() / 2;
    Eigen::VectorXd secondDiff(2 * (satNum - 1));
    for (unsigned int i = 0; i < (satNum - 1); ++i) {
        const unsigned int fdIndex = (i < refSatIndex ? i : i + 1);
        secondDiff(i) = firstDiff[fdIndex] - firstDiff[refSatIndex];
        secondDiff(satNum - 1 + i) = firstDiff[satNum + fdIndex] - firstDiff[satNum + refSatIndex];
    }
    return secondDiff;
}

FilterData recalcFilterData(const FilterData& filterData, const std::vector<std::string>& prevSatOrder,
                            const std::vector<std::string>& satOrder, const std::vector<double>& singleDiff) {
    assert(singleDiff.size() == 2 * satOrder.size());
    const unsigned int satNum = satOrder.size();
    const unsigned int dim = 3 + satNum;
    Eigen::VectorXd x(dim);
    Eigen::MatrixXd P(dim, dim);
    x(0) = filterData.x(0);
    x(1) = filterData.x(1);
    x(2) = filterData.x(2);
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            P(i, j) = filterData.P(i, j);
        }
    }
    std::vector<unsigned int> prevIndexes;  // индексы оставшихся спутников в старом prevSatOrder
    std::vector<unsigned int> newIndexes;  // индексы оставшихся спутников в новом satOrder
    for (unsigned int i = 0; i < satNum; ++i) {
        // определяем, был ли спутник на предыдущем шаге
        unsigned int satInd;
        for (satInd = 0; (satInd < prevSatOrder.size()) && (prevSatOrder[satInd] != satOrder[i]); ++satInd)
            ;
        if (satInd == prevSatOrder.size()) {
            // спутник новый
            x(3 + i) = singleDiff[satNum + i] - singleDiff[i];
            for (unsigned int j = 0; j < dim; ++j) {
                P(3 + i, j) = 0;
                P(j, 3 + i) = 0;
            }
            P(3 + i, 3 + i) = Constants::sdInitialValue;
        } else {
            // спутник уже был
            // вектор состояния
            x(3 + i) = filterData.x(3 + satInd);
            // первые 3 элемента матрицы P
            P(3 + i, 0) = filterData.P(3 + satInd, 0);
            P(3 + i, 1) = filterData.P(3 + satInd, 1);
            P(3 + i, 2) = filterData.P(3 + satInd, 2);
            P(0, 3 + i) = filterData.P(0, 3 + satInd);
            P(1, 3 + i) = filterData.P(1, 3 + satInd);
            P(2, 3 + i) = filterData.P(2, 3 + satInd);

            // остальные элементы
            for (unsigned int j = 0; j < newIndexes.size(); ++j) {
                // заполняем только те элементы, спутники которых уже были на предыдущем шаге
                P(3 + i, 3 + newIndexes[j]) = filterData.P(3 + satInd, 3 + prevIndexes[j]);
                P(3 + newIndexes[j], 3 + i) = filterData.P(3 + prevIndexes[j], 3 + satInd);
            }

            // диагональный элемент
            P(3 + i, 3 + i) = filterData.P(satInd + 3, satInd + 3);
            prevIndexes.push_back(satInd);
            newIndexes.push_back(i);
        }
    }
    return FilterData{x, P};
}

std::vector<Eigen::Vector3d> getSatEphemeris(
    const std::vector<std::string>& satOrder,
    const std::unordered_map<std::string, MatchedSatelliteMeasurements>& matchedSats) {
    std::vector<Eigen::Vector3d> res;
    res.reserve(satOrder.size());
    for (const auto& sat : satOrder) {
        res.push_back(matchedSats.at(sat).position);
    }
    return res;
}

Eigen::VectorXd calcMeasurement(const Eigen::VectorXd& x, const std::vector<std::string>& satOrder,
                                const std::unordered_map<std::string, MatchedSatelliteMeasurements>& matchedSats,
                                const unsigned int refSatIndex, const Eigen::Vector3d& stationPos) {
    const unsigned int satNum = satOrder.size();
    Eigen::VectorXd res(2 * satNum - 2);
    const MatchedSatelliteMeasurements& refSat = matchedSats.at(satOrder.at(refSatIndex));
    const double rir =
        std::hypot(refSat.position.x() - stationPos.x() - x(0), refSat.position.y() - stationPos.y() - x(1),
                   refSat.position.z() - stationPos.z() - x(2));
    const double rib = std::hypot(refSat.position.x() - stationPos.x(), refSat.position.y() - stationPos.y(),
                                  refSat.position.z() - stationPos.z());
    const double rirb = rir - rib;
    for (unsigned int i = 0; i < (satNum - 1); ++i) {
        const unsigned int satIndex = i < refSatIndex ? i : i + 1;
        const MatchedSatelliteMeasurements& sat = matchedSats.at(satOrder.at(satIndex));
        const double rjr =
            std::hypot(sat.position.x() - stationPos.x() - x(0), sat.position.y() - stationPos.y() - x(1),
                       sat.position.z() - stationPos.z() - x(2));
        const double rjb = std::hypot(sat.position.x() - stationPos.x(), sat.position.y() - stationPos.y(),
                                      sat.position.z() - stationPos.z());
        const double rjrb = rjr - rjb;
        const double rjirb = rjrb - rirb;
        res(i) = rjirb;
        res(satNum - 1 + i) = rjirb + Constants::waveL1 * (x(3 + satIndex) - x(3 + refSatIndex));
    }
    return res;
}

}  // namespace gnss