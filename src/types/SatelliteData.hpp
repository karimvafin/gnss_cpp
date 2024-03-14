//
// Created by Карим Вафин on 28.12.2023.
//

#ifndef GNSS_CPP_SATELLITEDATA_HPP
#define GNSS_CPP_SATELLITEDATA_HPP

#include <string>

namespace gnss {

struct SatelliteData {
    std::string name;
    double C1W, C2W;
};

}  // namespace gnss
#endif  // GNSS_CPP_SATELLITEDATA_HPP
