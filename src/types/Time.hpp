//
// Created by Карим Вафин on 28.12.2023.
//

#ifndef GNSS_CPP_TIME_HPP
#define GNSS_CPP_TIME_HPP

namespace gnss {

struct CalendarTime {
    unsigned int year;
    unsigned int month;
    unsigned int day;
    unsigned int hour;
    unsigned int minute;
    double second;
};

}  // namespace gnss

#endif  // GNSS_CPP_TIME_HPP
