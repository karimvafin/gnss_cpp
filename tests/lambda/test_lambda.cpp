/*------------------------------------------------------------------------------
* rtklib unit test driver : lambda/mlambda integer least square
*-----------------------------------------------------------------------------*/
#include "src/lambda/Lambda.hpp"
#include "gtest/gtest.h"
#include "LambdaTestData.hpp"
#include "src/utils/ContainerConvertion.hpp"
using namespace lambda_test_data;
using namespace gnss;

TEST(LAMBDA_TEST, LAMBDA_TEST1)
{
    int i,j,n,m,info;
    double F[6*2],s[2];
    
    n=6; m=2;
    info=lambda(n,m,a1,Q1,F,s);
    assert(info==0);

    for (j=0;j<m;j++) {
        for (i=0;i<n;i++) {
            assert(fabs(F[i+j*n]-F1[j+i*m])<1E-4);
        }
        assert(fabs(s[j]-s1[j])<1E-4);
    }
    printf("%s test1 : OK\n",__FILE__);
}

TEST(LAMBDA_TEST, LAMBDA_TEST2)
{
    int i,j,n,m,info;
    double F[10*2],s[2];
    
    n=10; m=2;
    info=lambda(n,m,a2,Q2,F,s);
    assert(info==0);
    
    for (j=0;j<m;j++) {
        for (i=0;i<n;i++) {
            assert(fabs(F[i+j*n]-F2[j+i*m])<1E-4);
        }
        assert(fabs(s[j]-s2[j])<1E-4);
    }
    printf("%s test2 : OK\n",__FILE__);
}

TEST(LAMBDA_TEST, LAMBDA_TEST3)
{

    const int n=10;
    const int m=2;
    Eigen::VectorXd aEigenVec = Eigen::Map<Eigen::VectorXd>(a2, n);
    Eigen::MatrixXd QEigenMat = Eigen::Map<Eigen::MatrixXd>(Q2, n, n);

    auto [F, s] = lambda(n, m, aEigenVec, QEigenMat);

    for (int j=0;j<m;j++) {
        for (int i=0;i<n;i++) {
            assert(fabs(F(i, j)-F2[j+i*m])<1E-4);
        }
        assert(fabs(s[j]-s2[j])<1E-4);
    }
    printf("%s test3 : OK\n",__FILE__);
}

TEST(LAMBDA_TEST, LAMBDA_TEST4)
{
    // тест из FAQ по лямбда алгоритму https://www.researchgate.net/publication/227304422_LAMBDA_FAQs
    const int n=3;
    const int m=2;

    Eigen::Vector3d aEigenVec = {5.450, 3.100, 2.970};
    Eigen::Matrix3d QEigenMat {{6.290, 5.978, 0.544},
                                 {5.978, 6.292, 2.340},
                                 {0.544, 2.340, 6.288}};

    Eigen::Vector3i aEigenVecTilda = {5, 3, 4};
    Eigen::Matrix3d Z {{-2, 3, 1},
                      {3, -3, -1},
                      {-1, 1, 0}};
    auto [F, s] = lambda(n, m, aEigenVec, QEigenMat);
    for (int i=0;i<n;++i) {
        assert(F(i, 0) == aEigenVecTilda(i));
    }

    std::cout << s(0) <<" "<< s(1) << std::endl;
    printf("%s test4 : OK\n",__FILE__);
}
