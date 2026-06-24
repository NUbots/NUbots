find_package(Boost REQUIRED)

# Build our NUClear module
nuclear_module(LIBRARIES Boost::headers)
