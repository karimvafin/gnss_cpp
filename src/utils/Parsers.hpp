//
// Created by Арсений Плахотнюк on 14.03.2024.
//

#ifndef GNSS_CPP_PARSERS_HPP
#define GNSS_CPP_PARSERS_HPP
#include <vector>
#include "src/types/InputTypes.hpp"
#include <fstream>
#include "third_party/json/single_include/nlohmann/json.hpp"
#include "ContainerConvertion.hpp"

namespace gnss::utils{
    /** Функция парсит json файл с данными rinex
     *
     * @param dataFileName - путь до json файла
     * @return
     */
    [[nodiscard]] std::optional<std::vector<RinexData>> parseRinex(const std::string &dataFileName);
    /** Функция парсит json файл с данными sp3
     *
     * @param dataFileName - путь до json файла
     * @return
     */
    [[nodiscard]] std::optional<std::vector<Sp3Data>> parseSp3(const std::string &dataFileName);
}

#endif  // GNSS_CPP_PARSERS_HPP
