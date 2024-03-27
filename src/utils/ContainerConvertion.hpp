//
// Created by Арсений Плахотнюк on 15.03.2024.
//

#ifndef GNSS_CPP_CONTAINERCONVERTION_HPP
#define GNSS_CPP_CONTAINERCONVERTION_HPP
#include "third_party/eigen/Eigen/Core"
namespace gnss::utils{
inline Eigen::Vector3d stlToEigenVector(const std::vector<double> &stlVector) {
    const unsigned int vecSize = stlVector.size();
    Eigen::VectorXd vec(vecSize);
    for(int i = 0; i < vecSize; ++i) {
        vec[i] = stlVector[i];
    }
    return vec;
}
inline Eigen::MatrixXd cArrayToMatrixXd(const unsigned n, const unsigned m, const double *a){
    Eigen::MatrixXd result;
    result.resize(n, m);
    for (int j=0;j<m;j++) {
        for (int i=0;i<n;i++) {
            result(i, j) =  a[i+j*n];
        }
    }
    return result;
}

inline Eigen::VectorXd cArrayToVectorXd(const unsigned n, const double *a){
    Eigen::VectorXd result;
    result.resize(n, 1);
    for (int i=0;i<n;i++) {
            result(i) =  a[i];
    }
    return result;
}
}

#endif  // GNSS_CPP_CONTAINERCONVERTION_HPP
