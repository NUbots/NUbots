SET(module_name "DarwinSensors")

FILE(GLOB src "src/*.cpp" "src/*.h")

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/src)

SOURCE_GROUP(Modules\\${module_name} FILES ${src})

ADD_LIBRARY(${module_name} ${src})