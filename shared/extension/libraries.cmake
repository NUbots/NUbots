find_package(yaml-cpp REQUIRED)

target_link_libraries(nuclear_extension PUBLIC yaml-cpp)

target_compile_features(nuclear_extension PUBLIC cxx_std_17)
