//
// Created by Арсений Плахотнюк on 15.03.2024.
//
#include <fstream>
#include <vector>
#include "gtest/gtest.h"
#include "types/Epoch.hpp"
#include "utils/Parsers.hpp"
using namespace gnss;

class DataParseSp3: public ::testing::Test{
protected:
    const std::string FILE_PATH = __FILE__;
    const std::string dataDirPAth = FILE_PATH.substr(0, FILE_PATH.size() - 32) + "/data/";
};

TEST_F(DataParseSp3, TEST1){
    const std::string filename = dataDirPAth + "satellite_data.json";
    auto data = utils::parseSp3(filename).value();
    std::fstream file(filename);
    nlohmann::json jsonData = nlohmann::json::parse(file);
    ASSERT_EQ(jsonData.size(), data.size());
    for(auto &&el: data[0].satelliteData){
        std::cout << "satellite: " << el.first << "; "
                  << "meas: " << el.second.x() << ", "<< el.second.y() << el.second.z() << std::endl;
    }
}



