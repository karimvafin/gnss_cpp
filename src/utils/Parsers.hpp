//
// Created by Арсений Плахотнюк on 14.03.2024.
//

#ifndef GNSS_CPP_PARSERS_HPP
#define GNSS_CPP_PARSERS_HPP
#include <vector>
#include "src/types/Epoch.hpp"
#include <fstream>
#include "third_party/json/single_include/nlohmann/json.hpp"
#include "ContainerConvertion.hpp"

namespace gnss::utils{
    /** Функция парсит json файл с данными rinex
     *
     * @param dataFileName - путь до json файла
     * @return
     */
    [[nodiscard]] std::optional<std::vector<RinexData>> parseRinex(const std::string &dataFileName){
        std::fstream file(dataFileName);
        nlohmann::json jsonData = nlohmann::json::parse(file);
        std::vector<RinexData> epochsData;
        epochsData.reserve(jsonData.size());
        uint satNum;
        for(auto it = jsonData.begin(), itend = jsonData.end(); it != itend; ++it){
            auto epochData = it.value();
            satNum = epochData["sv"].size();
            std::vector<std::pair<std::string, PhaseCodeMeas>> satelliteData;
            satelliteData.reserve(satNum);
            for(uint i = 0; i < satNum; ++i){
                satelliteData.emplace_back(
                    epochData["sv"][i].get<std::string>(),
                    PhaseCodeMeas{.C1W = epochData["C1W"][i].get<double>(), .L1W = epochData["L1W"][i].get<double>()});
            }
            epochsData.push_back({.timeJD = strToJd(it.key()).value(), .satelliteData = satelliteData});
        }
        return epochsData;
    }
    /** Функция парсит json файл с данными sp3
     *
     * @param dataFileName - путь до json файла
     * @return
     */
    [[nodiscard]] std::optional<std::vector<Sp3Data>> parseSp3(const std::string &dataFileName){
        std::fstream file(dataFileName);
        nlohmann::json jsonData = nlohmann::json::parse(file);
        std::vector<Sp3Data> epochsData;
        epochsData.reserve(jsonData.size());
        uint satNum;
        std::vector<double> pos;
        for(auto it = jsonData.begin(), itend = jsonData.end(); it != itend; ++it){
            auto epochData = it.value();
            satNum = epochData["sv"].size();
            std::vector<std::pair<std::string, Eigen::Vector3d>> satelliteData;
            satelliteData.reserve(satNum);
            for(uint i = 0; i < satNum; ++i){
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

#endif  // GNSS_CPP_PARSERS_HPP
