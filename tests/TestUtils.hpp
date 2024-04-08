//
// Created by Карим Вафин on 08.04.2024.
//

#ifndef GNSS_CPP_TESTUTILS_HPP
#define GNSS_CPP_TESTUTILS_HPP

#include "Eigen/Core"
inline bool assertVectorEqual(const Eigen::VectorXd& vec1, const Eigen::VectorXd& vec2) {
    if (vec1.size() != vec2.size()) return false;
    for (int i = 0; i < vec1.size(); ++i) {
        if (vec2(i) == 0 && vec2(i) != 0) return false;
        if (std::abs((vec1(i) - vec2(i)) / vec1(i)) > 1e-15) return false;
    }
    return true;
}

inline bool assertMatrixEqual(const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2) {
    if (mat1.rows() != mat2.rows()) return false;
    if (mat1.cols() != mat2.cols()) return false;
    for (int i = 0; i < mat1.rows(); ++i)
        for (int j = 0; j < mat1.cols(); ++j) {
            if (mat2(i, j) == 0 && mat1(i, j) != 0) return false;
            if (std::abs((mat1(i, j) - mat2(i, j)) / mat2(i, j)) > 1e-15) return false;
        }
    return true;
}
#endif  // GNSS_CPP_TESTUTILS_HPP
