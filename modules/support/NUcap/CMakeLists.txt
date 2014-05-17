# Find the BOOST Library
FIND_PACKAGE(Boost COMPONENTS system thread REQUIRED)

# Build our NUClear module
NUCLEAR_MODULE(INCLUDES ${Boost_INCLUDE_DIRS}
               LIBRARIES ${Boost_LIBRARIES})