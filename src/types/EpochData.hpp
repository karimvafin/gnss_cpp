//
// Created by Карим Вафин on 28.12.2023.
//

#ifndef GNSS_CPP_EPOCHDATA_HPP
#define GNSS_CPP_EPOCHDATA_HPP

#include <vector>

#include "Time.hpp"
#include "SatelliteData.hpp"

namespace gnss {

struct EpochData {
    CalendarTime epoch;
    std::vector<SatelliteData> measurements;
};

}  // namespace gnss

#endif //GNSS_CPP_EPOCHDATA_HPP
