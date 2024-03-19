//
// Created by vladimir on 19.03.24.
//

#ifndef GNSS_CPP_CYCLESLIPDETECTION_HPP
#define GNSS_CPP_CYCLESLIPDETECTION_HPP

#include <deque>
#include <cmath>

class SlidingWindow {
private:
    unsigned int maxSize_;
    unsigned int currentSize_;
    std::deque<double> slidingWindow_;
public:
    explicit SlidingWindow(const unsigned int maxSize) : maxSize_(maxSize), currentSize_(0) {}

    void push_front(const double el) {
        if (currentSize_ > maxSize_) {
            slidingWindow_.pop_back();
            slidingWindow_.push_front(el);
        } else {
            slidingWindow_.push_front(el);
            currentSize_++;
        }
    }

    [[nodiscard]] double getCurrentSize() const {
        return currentSize_;
    }

    void setCurrentSize(const unsigned int newCurrentSize) {
        currentSize_ = newCurrentSize;
    }

    double getSum() {
        double sum = 0;
        for (const auto el : slidingWindow_) {
            sum += el;
        }
        return sum;
    }

    double getSumOfSquares() {
        double sum = 0;
        for (const auto el : slidingWindow_) {
            sum += el * el;
        }
        return sum;
    }
};

CycleSlipData determineCycleSlip(const std::vector<double> pseudorange,
                                 const std::vector<double> carrierPhase,
                                 const std::vector<double> epochs,
                                 const double tolDeltaT) {
    SlidingWindow slidingWindow(100);
    for (unsigned int i = 1; i < pseudorange.size() - 1; i++) {
        if (epochs[i] - epochs[i - 1] > tolDeltaT) {
            return {1, epochs[i], true};
        }
        slidingWindow.push_front(carrierPhase[i] - pseudorange[i]);
        const double mean = slidingWindow.getSum() / slidingWindow.getCurrentSize();
        const double meanSquares = slidingWindow.getSumOfSquares() / slidingWindow.getCurrentSize();
        const double S = std::sqrt(meanSquares - mean * mean);
        if (std::abs(carrierPhase[i] - pseudorange[i] - mean) > 6 * S) {
            return {1, epochs[i], true};
        }
    }
    return {std::nullopt, std::nullopt, false};
}

#endif  // GNSS_CPP_CYCLESLIPDETECTION_HPP
