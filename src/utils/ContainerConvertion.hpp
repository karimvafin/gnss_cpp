//
// Created by Арсений Плахотнюк on 15.03.2024.
//

#ifndef GNSS_CPP_CONTAINERCONVERTION_HPP
#define GNSS_CPP_CONTAINERCONVERTION_HPP
#include "third_party/eigen/Eigen/Core"
namespace gnss::utils{
inline Eigen::Vector3d stlToEigenVector(const std::vector<double> &stlVector) {
    if(stlVector.size() != 3){
        return Eigen::Vector3d::Zero();
    }
    const int vecSize = stlVector.size();
    Eigen::VectorXd vec(vecSize);
    for(int i = 0; i < vecSize; ++i) {
        vec[i] = stlVector[i];
    }
    return vec;
}
}

#endif  // GNSS_CPP_CONTAINERCONVERTION_HPP
