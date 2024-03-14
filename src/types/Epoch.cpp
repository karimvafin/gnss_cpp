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

}