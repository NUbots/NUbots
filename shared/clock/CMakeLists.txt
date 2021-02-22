# Create the clock library
add_library(nubots_clock STATIC clock.cpp clock.hpp)
target_compile_definitions(nubots_clock PUBLIC NUCLEAR_CUSTOM_CLOCK)
set_property(TARGET nubots_clock PROPERTY POSITION_INDEPENDENT_CODE ON)

# Link such that anything that is using the nuclear library will get this library too
target_link_libraries(nuclear_message PUBLIC nubots_clock)
target_link_libraries(nuclear_utility PUBLIC nubots_clock)
