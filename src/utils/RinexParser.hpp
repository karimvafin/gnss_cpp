//
// Created by Арсений Плахотнюк on 14.03.2024.
//

#ifndef GNSS_CPP_RINEXPARSER_HPP
#define GNSS_CPP_RINEXPARSER_HPP
#include <vector>
#include "src/types/Epoch.hpp"
#include <fstream>
#include "third_party/json/single_include/nlohmann/json.hpp"

namespace gnss::utils{
    std::vector<EpochData> parseRinex(const std::string &dataFileName){
        std::fstream file(dataFileName);
        nlohmann::json jsonData = nlohmann::json::parse(file);
        std::vector<EpochData> epochsData;
        epochsData.reserve(jsonData.size());
        uint satNum;
        for(auto it = jsonData.begin(), itend = jsonData.end(); it != itend; ++it){
            satNum = it.value()["sv"].size();
            std::vector<std::pair<std::string, Measurements>> satelliteData;
            satelliteData.reserve(satNum);
            auto epochData = it.value();
            for(uint i = 0; i < satNum; ++i){
                satelliteData.emplace_back(it.value()["sv"][i].get<std::string>(),
                                  Measurements{.C1W = epochData["C1W"][i].get<double>(),
                                      .L1W = epochData["L1W"][i].get<double>()});
            }
            epochsData.push_back({.time = fromString(it.key()).value(), .satelliteData = satelliteData});
        }
        return epochsData;
    }
}

#endif  // GNSS_CPP_RINEXPARSER_HPP
