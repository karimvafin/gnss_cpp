//
// Created by vladimir on 19.03.24.
//

#ifndef GNSS_CPP_CYCLESLIPDETECTION_HPP
#define GNSS_CPP_CYCLESLIPDETECTION_HPP

#include "CycleSlipData.hpp"

#include <cmath>
#include <deque>
#include <vector>

class SlidingWindow {
private:
    unsigned int maxSize_;
    unsigned int currentSize_;
    std::deque<double> slidingWindow_;

public:
    explicit SlidingWindow(const unsigned int maxSize) : maxSize_(maxSize), currentSize_(0) {}

    void push_front(const double el);

    [[nodiscard]] double getCurrentSize() const;

    void setCurrentSize(const unsigned int newCurrentSize);

    [[nodiscard]] double getSum() const;

    [[nodiscard]] double getSumOfSquares() const;
};

[[nodiscard]] CycleSlipData determineCycleSlip(const std::vector<double> pseudorange,
                                               const std::vector<double> carrierPhase, const std::vector<double> epochs,
                                               const double tolDeltaT) noexcept;

#endif  // GNSS_CPP_CYCLESLIPDETECTION_HPP
