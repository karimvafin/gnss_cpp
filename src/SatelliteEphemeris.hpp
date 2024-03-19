//
// Created by Карим Вафин on 14.03.2024.
//

#ifndef GNSS_CPP_SATELLITEEPHEMERIS_HPP
#define GNSS_CPP_SATELLITEEPHEMERIS_HPP

#include <optional>

#include "src/types/Epoch.hpp"
#include "src/types/ephemeris/Ephemeris.hpp"
#include "third_party/eigen/Eigen/Core"

namespace gnss {

std::optional<Eigen::Vector3d> interpolateSatelliteEphemeris(const std::string &satName, double epochJd,
                                                             const std::vector<EpochEphemeris> &ephemeris);

}  // namespace gnss

#endif  // GNSS_CPP_SATELLITEEPHEMERIS_HPP
