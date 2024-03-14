//
// Created by Карим Вафин on 14.03.2024.
//
#include <optional>

#include "Epoch.hpp"
#include "third_party/sofa/src/sofa.h"

namespace gnss {

std::optional<double> toJd(const Epoch& epoch) {
    double jdDay, jdPart;
    int res = iauDtf2d("TT", static_cast<int>(epoch.year), static_cast<int>(epoch.month), static_cast<int>(epoch.day),
                       static_cast<int>(epoch.hour), static_cast<int>(epoch.minute), epoch.second, &jdDay, &jdPart);
    if (res != 0) return std::nullopt;
    return jdDay + jdPart;
}

/**
 *
 * @param date - required time format: YYYY-MM-DDTHH:MM:SS.SSSSSSSSS
 * @return
 */
std::optional<Epoch> fromString(const std::string &date){
    return Epoch{.year = static_cast<decltype(Epoch::year)>(std::stoi(date.substr(0, 4))),
            .month = static_cast<decltype(Epoch::month)>(std::stoi(date.substr(5, 7))),
            .day = static_cast<decltype(Epoch::day)>(std::stoi(date.substr(8, 10))),
            .hour = static_cast<decltype(Epoch::hour)>(std::stoi(date.substr(11, 13))),
            .minute = static_cast<decltype(Epoch::minute)>(std::stoi(date.substr(14, 16))),
            .second = static_cast<decltype(Epoch::second)>(std::stoi(date.substr(17)))};
}

}