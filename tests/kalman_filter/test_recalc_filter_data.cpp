//
// Created by Карим Вафин on 07.04.2024.
//
#include "gtest/gtest.h"

#include "src/process_sat_data/ProcessSatData.hpp"
#include "tests/TestUtils.hpp"

#include <iostream>

TEST(RECALC_FILTER_DATA, SAME_SATS) {
    const unsigned int satNum = 3;
    const std::vector<std::string> prevSatOrder = {"G3", "G5", "G6"};
    const std::vector<std::string> satOrder = {"G3", "G5", "G6"};
    const std::vector<double> singleDiff = {22.2, 33.3, 21.2, 25.2, 39.3, 25.2};
    Eigen::VectorXd x{{1.2, 1.1, 2.3, 1.4, 0.5, 6.7}};
    Eigen::MatrixXd P{{3, 2, 1, 2, 4, 5}, {6, 3, 4, 1, 4, 6}, {7, 8, 2, 4, 4, 2},
                      {9, 1, 1, 1, 2, 3}, {3, 3, 2, 3, 2, 1}, {4, 4, 2, 3, 6, 6}};
    const gnss::FilterData recalc = gnss::recalcFilterData(gnss::FilterData{x, P}, prevSatOrder, satOrder, singleDiff);

    ASSERT_TRUE(assertVectorEqual(x, recalc.x));
    ASSERT_TRUE(assertMatrixEqual(P, recalc.P));
}

TEST(RECALC_FILTER_DATA, PLUS_ONE_SAT) {
    const std::vector<std::string> prevSatOrder = {"G3", "G5", "G6"};
    const std::vector<std::string> satOrder = {"G3", "G5", "G18", "G6"};
    const std::vector<double> singleDiff = {22.2, 33.3, 12.3, 21.2, 25.2, 17.8, 39.3, 25.2};
    Eigen::VectorXd x{{1.2, 1.1, 2.3, 1.4, 0.5, 6.7}};
    Eigen::MatrixXd P{{3, 2, 1, 2, 4, 5}, {6, 3, 4, 1, 4, 6}, {7, 8, 2, 4, 4, 2},
                      {9, 1, 1, 1, 2, 3}, {3, 3, 2, 3, 2, 1}, {4, 4, 2, 3, 6, 6}};
    const gnss::FilterData recalc = gnss::recalcFilterData(gnss::FilterData{x, P}, prevSatOrder, satOrder, singleDiff);

    const Eigen::VectorXd xRef{{1.2, 1.1, 2.3, 1.4, 0.5, 27, 6.7}};
    const Eigen::MatrixXd pRef{{3, 2, 1, 2, 4, 0, 5}, {6, 3, 4, 1, 4, 0, 6}, {7, 8, 2, 4, 4, 0, 2},
                               {9, 1, 1, 1, 2, 0, 3}, {3, 3, 2, 3, 2, 0, 1}, {0, 0, 0, 0, 0, 1e+10, 0},
                               {4, 4, 2, 3, 6, 0, 6}};

    ASSERT_TRUE(assertVectorEqual(recalc.x, xRef));
    ASSERT_TRUE(assertMatrixEqual(recalc.P, pRef));
}

TEST(RECALC_FILTER_DATA, NEW_SATS) {
    const std::vector<std::string> prevSatOrder = {"G3", "G5", "G6"};
    const std::vector<std::string> satOrder = {"G4", "G7", "G18", "G9"};
    const std::vector<double> singleDiff = {22.2, 33.3, 12.3, 21.2, 25.2, 17.8, 39.3, 25.2};
    Eigen::VectorXd x{{1.2, 1.1, 2.3, 1.4, 0.5, 6.7}};
    Eigen::MatrixXd P{{3, 2, 1, 2, 4, 5}, {6, 3, 4, 1, 4, 6}, {7, 8, 2, 4, 4, 2},
                      {9, 1, 1, 1, 2, 3}, {3, 3, 2, 3, 2, 1}, {4, 4, 2, 3, 6, 6}};
    const gnss::FilterData recalc = gnss::recalcFilterData(gnss::FilterData{x, P}, prevSatOrder, satOrder, singleDiff);

    const Eigen::VectorXd xRef{{1.2, 1.1, 2.3, singleDiff[4] - singleDiff[0], singleDiff[5] - singleDiff[1],
                                singleDiff[6] - singleDiff[2], singleDiff[7] - singleDiff[3]}};
    const Eigen::MatrixXd pRef{{3, 2, 1, 0, 0, 0, 0},    {6, 3, 4, 0, 0, 0, 0},    {7, 8, 2, 0, 0, 0, 0},
                               {0, 0, 0, 1e10, 0, 0, 0}, {0, 0, 0, 0, 1e10, 0, 0}, {0, 0, 0, 0, 0, 1e10, 0},
                               {0, 0, 0, 0, 0, 0, 1e10}};

    ASSERT_TRUE(assertVectorEqual(recalc.x, xRef));
    ASSERT_TRUE(assertMatrixEqual(recalc.P, pRef));
}

TEST(RECALC_FILTER_DATA, CHANGED_ORDER) {
    const std::vector<std::string> prevSatOrder = {"G3", "G5", "G6"};
    const std::vector<std::string> satOrder = {"G5", "G7", "G3", "G6"};
    const std::vector<double> singleDiff = {22.2, 33.3, 12.3, 21.2, 25.2, 17.8, 39.3, 25.2};
    Eigen::VectorXd x{{1.2, 1.1, 2.3, 1.4, 0.5, 6.7}};
    Eigen::MatrixXd P{{3, 2, 1, 2, 4, 5}, {6, 3, 4, 1, 4, 6}, {7, 8, 2, 4, 4, 2},
                      {9, 1, 1, 1, 2, 3}, {3, 3, 2, 3, 2, 1}, {4, 4, 2, 3, 6, 6}};
    const gnss::FilterData recalc = gnss::recalcFilterData(gnss::FilterData{x, P}, prevSatOrder, satOrder, singleDiff);

    const Eigen::VectorXd xRef{{1.2, 1.1, 2.3, 0.5, singleDiff[5] - singleDiff[1],
                                1.4, 6.7}};
    const Eigen::MatrixXd pRef{{3, 2, 1, 4, 0, 2, 5}, {6, 3, 4, 4, 0, 1, 6},    {7, 8, 2, 4, 0, 4, 2},
                               {3, 3, 2, 2, 0, 3, 1}, {0, 0, 0, 0, 1e10, 0, 0}, {9, 1, 1, 2, 0, 1, 3},
                               {4, 4, 2, 6, 0, 3, 6}};

    ASSERT_TRUE(assertVectorEqual(recalc.x, xRef));
    ASSERT_TRUE(assertMatrixEqual(recalc.P, pRef));
}