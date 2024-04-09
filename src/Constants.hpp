//
// Created by Карим Вафин on 27.03.2024.
//

#ifndef GNSS_CPP_CONSTANTS_HPP
#define GNSS_CPP_CONSTANTS_HPP

namespace gnss::Constants {

constexpr double freqL1 = 1575.42e9;  // Hz
constexpr double lightVelocity = 299792458;  // m/s
constexpr double waveL1 = lightVelocity / freqL1;  // m
constexpr double secondsInDay = 86400.0;
constexpr double sdInitialValue = 1e5;
constexpr double sigmaPosition = 1e9;
constexpr double metersInKm = 1000.0;

}
#endif  // GNSS_CPP_CONSTANTS_HPP
