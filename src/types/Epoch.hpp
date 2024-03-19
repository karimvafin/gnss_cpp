//
// Created by Карим Вафин on 14.03.2024.
//

#ifndef GNSS_CPP_EPOCH_HPP
#define GNSS_CPP_EPOCH_HPP

#include <array>
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>
#include "Eigen/Dense"

namespace gnss {

struct Epoch {
    unsigned int year;
    unsigned int month;
    unsigned int day;
    unsigned int hour;
    unsigned int minute;
    double second;
};

std::optional<double> toJd(const Epoch& epoch);

std::optional<Epoch> fromString(const std::string &date);

std::optional<double> strToJd(const std::string &date);


struct PhaseCodeMeas{
    double C1W;
    double L1W;
};

struct RinexData{
    double timeJD;
    std::vector<std::pair<std::string, PhaseCodeMeas>> satelliteData;
};

struct Sp3Data{
    double timeJD;
    std::vector<std::pair<std::string, Eigen::Vector3d>> satelliteData;
};



}  // namespace gnss

#endif  // GNSS_CPP_EPOCH_HPP
