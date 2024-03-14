file(GLOB_RECURSE SOURCES *.h *.cpp *.hpp *.cc)

set(PROJECT_LIB_NAME gnss_cpp_lib)

add_library(${PROJECT_LIB_NAME} STATIC ${SOURCES})

set(TARGET_LIBRS eigen sofa nlohmann_json)

target_link_libraries(${PROJECT_LIB_NAME} PUBLIC ${TARGET_LIBRS})
target_include_directories(${PROJECT_LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/..")

