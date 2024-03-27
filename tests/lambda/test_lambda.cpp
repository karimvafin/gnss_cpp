/*------------------------------------------------------------------------------
* rtklib unit test driver : lambda/mlambda integer least square
*-----------------------------------------------------------------------------*/
#include "src/lambda/Lambda.hpp"
#include "gtest/gtest.h"
#include "LambdaTestData.hpp"
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
