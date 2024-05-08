//
// Created by Карим Вафин on 27.03.2024.
//
#include "StateVector.hpp"

namespace gnss {

Eigen::Vector3d getPosition(const Eigen::VectorXd& state) {
    assert(state.size() >= 3);
    return Eigen::Vector3d{state(0), state(1), state(2)};
}

}