//
// Created by vladimir on 19.03.24.
//

#ifndef GNSS_CPP_CYCLESLIPDATA_HPP
#define GNSS_CPP_CYCLESLIPDATA_HPP

#include <optional>

struct CycleSlipData {
    std::optional<unsigned int> PRN; // Пока без этой штуки, так как для одного спутника
    std::optional<double> time;
    bool isCycleSlipHappend;
};

#endif  // GNSS_CPP_CYCLESLIPDATA_HPP
