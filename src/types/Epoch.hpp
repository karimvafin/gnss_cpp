//
// Created by Карим Вафин on 14.03.2024.
//

#ifndef GNSS_CPP_EPOCH_HPP
#define GNSS_CPP_EPOCH_HPP

#include <array>

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

}  // namespace gnss

#endif  // GNSS_CPP_EPOCH_HPP
