# Build the logging utility library
find_package(nlohmann_json REQUIRED)

add_library(logging JsonLogger.cpp)
target_link_libraries(logging PUBLIC nuclear nlohmann_json::nlohmann_json)
target_include_directories(logging PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})