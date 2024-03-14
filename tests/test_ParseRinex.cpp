//
// Created by Арсений Плахотнюк on 14.03.2024.
//
#include "gtest/gtest.h"
#include "types/Epoch.hpp"
#include "third_party/json/single_include/nlohmann/json.hpp"
#include <fstream>
#include <vector>
using namespace gnss;

class Data: public ::testing::Test{
protected:
    const std::string FILE_PATH = __FILE__;
    const std::string dataDirPAth = FILE_PATH.substr(0, FILE_PATH.size() - 26) + "/data/";
};

TEST_F(Data, TEST1){
    std::fstream file(dataDirPAth + "rover_data.json");
    nlohmann::json jsonData = nlohmann::json::parse(file);

    auto it = jsonData.begin();
    std::string time = it.key();
    Epoch epoch = fromString(time).value();


}

TEST_F(Data, TEST2){
    std::fstream file(dataDirPAth + "rover_data.json");

    nlohmann::json jsonData = nlohmann::json::parse(file);

    auto it = jsonData.begin();
    std::string time = it.key();
    Epoch epochTime =  fromString(time).value();
    auto epochData = it.value();
//    EpochData epoch = {.time = epochTime,
//                       .satelliteData = std::vector<>({it.value()["sv"][0].get<std::string>(),
//                           Measurements{.C1W=it.value()["C1W"][0].get<double>(), .L1W = it.value()["L1W"][0].get<double>()}})};
    std::cout << epochData["sv"] << std::endl;
    std::cout << epochData["C1W"] << std::endl;
    std::cout << epochData["L1W"] << std::endl;
//    Measurements meas{.C1W = epochData["C1W"].get<double>(), .L1W = epochData["L1W"].get<double>()};
    std::vector<std::pair<std::string, Measurements>> data;
    data.reserve(it.value()["sv"].size());
    std::pair<std::string, Measurements> meas{it.value()["sv"][0].get<std::string>(),
        Measurements{.C1W = epochData["C1W"][0].get<double>(), .L1W = epochData["L1W"][0].get<double>()}};

    for(int i = 0; i < it.value()["sv"].size(); ++i){
        data.emplace_back(it.value()["sv"][i].get<std::string>(),
            Measurements{.C1W = epochData["C1W"][i].get<double>(), .L1W = epochData["L1W"][i].get<double>()});
    }

}
