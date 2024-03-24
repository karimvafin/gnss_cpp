//
// Created by Карим Вафин on 14.03.2024.
//
#include <algorithm>

#include "SatelliteEphemeris.hpp"
#include "src/utils/LagrangeInterpolation.hpp"

namespace gnss {

std::optional<Eigen::Vector3d> interpolateSatelliteEphemeris(const std::string& satName, const double epochJd,
                                                             const std::vector<EpochEphemeris>& ephemeris) {
    const auto comp = [](const EpochEphemeris& first, const double& second) { return first.epochJd < second; };
    const auto it0 = std::lower_bound(ephemeris.begin(), ephemeris.end(), epochJd, comp);
    std::vector<double> epochs;
    epochs.reserve(10);
    std::vector<Eigen::Vector3d> coords;
    coords.reserve(10);

    // заполняем то, что слева
    const int leftBorder = std::max(5, static_cast<int>(it0 - ephemeris.begin()));
    const int rightBorder = std::max(4, static_cast<int>(ephemeris.end() - it0));
    for (int i = -leftBorder; i < rightBorder; ++i) {
        const auto currIt = it0 + i;
        const auto satIt = currIt->satEphemeris.find(satName);
        if (satIt != currIt->satEphemeris.end()) {
            epochs.push_back(currIt->epochJd);
            coords.push_back(satIt->second);
        }
    }
    if (epochs.size() < 2) return std::nullopt;
    return lagrangeInterpolation(epochJd, epochs, coords);
}

Eigen::Vector3d correctEarthRotation(const Eigen::Vector3d& vec, const double deltaTime) {
    constexpr double earthAngularVel = 2 * M_PI / 86400;  // rad/s
    const double angle = earthAngularVel * deltaTime;
    const double cos = std::cos(angle);
    const double sin = std::sin(angle);
    return Eigen::Vector3d{vec.x() * cos - vec.y() * sin, vec.x() * sin + vec.y() * cos, vec.z()};
}

}  // namespace gnss