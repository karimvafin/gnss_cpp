#include <iostream>

#include "eigen/Eigen/Core"

int main() {
    Eigen::Vector<double, 3> a{1, 2, 3};
    std::cout << a << std::endl;
}