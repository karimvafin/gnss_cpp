//
// Created by Карим Вафин on 14.03.2024.
//

#ifndef GNSS_CPP_EPHEMERIS_HPP
#define GNSS_CPP_EPHEMERIS_HPP

#include <vector>
#include <unordered_map>

#include "third_party/eigen/Eigen/Core"
#include "src/types/Epoch.hpp"

namespace gnss {

struct EpochEphemeris {
    double epochJd;
    std::unordered_map<std::string, Eigen::Vector3d> satEphemeris;
};

}
#endif  // GNSS_CPP_EPHEMERIS_HPP
