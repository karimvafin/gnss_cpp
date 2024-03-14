#include <iostream>
#include <iomanip>

#include "eigen/Eigen/Core"
#include "sofa/src/sofa.h"

int main() {
    double jd0, jd1;
    int res = iauDtf2d("TT", 2023, 5, 17, 12, 0, 36.2325, &jd0, &jd1);
    std::cout << std::setprecision(15);
    std::cout << jd0 + jd1 << std::endl;
}