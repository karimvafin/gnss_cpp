//
// Created by vladimir on 19.03.24.
//

#include "gtest/gtest.h"

#include "src/cycle_slip_detection/CycleSlipData.hpp"
#include "src/cycle_slip_detection/CycleSlipDetection.hpp"

#include <random>

const unsigned int size = 10000;

std::vector<double> getPseudorange() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-0.1, 0.1);

    std::vector<double> pseudorange(size);
    for (unsigned int i = 0; i < pseudorange.size(); i++) {
        pseudorange[i] = 0.5 + dis(gen);
    }

    return pseudorange;
}

std::vector<double> getCarrierPhase() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-0.1, 0.1);

    std::vector<double> carrierPhase(size);
    for (unsigned int i = 0; i < carrierPhase.size(); i++) {
        carrierPhase[i] = 1.0 + dis(gen);
    }
    carrierPhase[1000] = 1.8;
    return carrierPhase;
}

std::vector<double> getEpochs() {
    std::vector<double> epochs(size);
    for (unsigned int i = 0; i < epochs.size(); i++) {
        epochs[i] = i;
    }
    return epochs;
}

class CYCLE_SLIP: public ::testing::Test{
protected:
    const std::vector<double> pseudorange = getPseudorange();
    const std::vector<double> carrierPhase = getCarrierPhase();
    const std::vector<double> epochs = getEpochs();
    const double tolDeltaT = 15;
};

TEST_F(CYCLE_SLIP, CYCLE_SLIP_DETECTION){
    const CycleSlipData cycleSlip = determineCycleSlip(pseudorange,
                                                       carrierPhase,
                                                       epochs,
                                                       tolDeltaT);

    ASSERT_TRUE(cycleSlip.PRN);
    ASSERT_TRUE(cycleSlip.time);
    ASSERT_TRUE(cycleSlip.isCycleSlipHappend);

    std::cout << "{" << ((cycleSlip.PRN == std::nullopt) ? "nullopt" : std::to_string(cycleSlip.PRN.value()))
              << ", " << ((cycleSlip.time == std::nullopt) ? "nullopt" : std::to_string(cycleSlip.time.value()))
              << ", " << (cycleSlip.isCycleSlipHappend ? "true" : "false") << "}" << std::endl;
}
