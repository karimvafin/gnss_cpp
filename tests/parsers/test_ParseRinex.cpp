//
// Created by Арсений Плахотнюк on 14.03.2024.
//
#include <fstream>
#include <vector>
#include "gtest/gtest.h"
#include "types/Epoch.hpp"
#include "utils/Parsers.hpp"
using namespace gnss;

class DataParseRinex: public ::testing::Test{
protected:
    const std::string FILE_PATH = __FILE__;
    const std::string dataDirPAth = FILE_PATH.substr(0, FILE_PATH.size() - 34) + "/data/";
};

TEST_F(DataParseRinex, TEST_ROVER){
    const std::string filename = dataDirPAth + "rover_data.json";
    auto data = utils::parseRinex(filename).value();
    std::fstream file(filename);
    nlohmann::json jsonData = nlohmann::json::parse(file);
    ASSERT_EQ(jsonData.size(), data.size());
    for(auto &&el: data[0].satelliteData){
        std::cout << "satellite: " << el.first << "; " << "meas: " << el.second.C1W << ", "<< el.second.L1W << std::endl;
    }

}

TEST_F(DataParseRinex, TEST_STATION){
    const std::string filename = dataDirPAth + "station_data.json";
    auto data = utils::parseRinex(filename).value();
    std::fstream file(filename);
    nlohmann::json jsonData = nlohmann::json::parse(file);
    ASSERT_EQ(jsonData.size(), data.size());
    for(auto &&el: data[0].satelliteData){
        std::cout << "satellite: " << el.first << "; " << "meas: " << el.second.C1W << ", "<< el.second.L1W << std::endl;
    }
}