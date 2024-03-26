//
// Created by Арсений Плахотнюк on 26.03.2024.
//

#ifndef GNSS_CPP_INPUTTYPES_HPP
#define GNSS_CPP_INPUTTYPES_HPP
#include <string>
#include <unordered_map>
#include "Eigen/Dense"

namespace gnss {

struct PhaseCodeMeas{
    double C1W;
    double L1W;
};

struct RinexData{
    double timeJD;
    std::unordered_map<std::string, PhaseCodeMeas>  satelliteData;
};

struct Sp3Data{
    double timeJD;
    std::unordered_map<std::string, Eigen::Vector3d> satelliteData;
};

}

#endif  // GNSS_CPP_INPUTTYPES_HPP
