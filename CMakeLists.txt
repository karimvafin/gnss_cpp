cmake_minimum_required(VERSION 3.23)
project(gnss_cpp)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(src)
add_subdirectory(third_party)

add_executable(gnss_cpp
        main.cpp)
target_include_directories(gnss_cpp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
target_link_libraries(gnss_cpp PUBLIC eigen)