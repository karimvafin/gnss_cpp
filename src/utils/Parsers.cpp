//
// Created by Арсений Плахотнюк on 19.03.2024.
//
#include "Parsers.hpp"
namespace gnss::utils{

[[nodiscard]] std::optional<std::vector<RinexData>> parseRinex(const std::string &dataFileName){
    std::fstream file(dataFileName);
    nlohmann::json jsonData = nlohmann::json::parse(file);
    std::vector<RinexData> epochsData;
    epochsData.reserve(jsonData.size());
    unsigned int satNum;
    for(auto it = jsonData.begin(), itend = jsonData.end(); it != itend; ++it){
        auto epochData = it.value();
        satNum = epochData["sv"].size();
        std::vector<std::pair<std::string, PhaseCodeMeas>> satelliteData;
        satelliteData.reserve(satNum);
        for(unsigned int i = 0; i < satNum; ++i){
            satelliteData.emplace_back(
                epochData["sv"][i].get<std::string>(),
                PhaseCodeMeas{.C1W = epochData["C1W"][i].get<double>(), .L1W = epochData["L1W"][i].get<double>()});
        }
        epochsData.push_back({.timeJD = strToJd(it.key()).value(), .satelliteData = satelliteData});
    }
    return epochsData;
}

[[nodiscard]] std::optional<std::vector<Sp3Data>> parseSp3(const std::string &dataFileName){
    std::fstream file(dataFileName);
    nlohmann::json jsonData = nlohmann::json::parse(file);
    std::vector<Sp3Data> epochsData;
    epochsData.reserve(jsonData.size());
    unsigned int satNum;
    std::vector<double> pos;
    for(auto it = jsonData.begin(), itend = jsonData.end(); it != itend; ++it){
        auto epochData = it.value();
        satNum = epochData["sv"].size();
        std::vector<std::pair<std::string, Eigen::Vector3d>> satelliteData;
        satelliteData.reserve(satNum);
        for(unsigned int i = 0; i < satNum; ++i){
            pos = epochData["position"][i].get<std::vector<double>>();
            if(pos.size() == 3) {
                satelliteData.emplace_back(
                    epochData["sv"][i].get<std::string>(),
                    utils::stlToEigenVector(pos));
            }
        }
        epochsData.push_back({.timeJD = strToJd(it.key()).value(), .satelliteData = satelliteData});
    }
    return epochsData;
}

}