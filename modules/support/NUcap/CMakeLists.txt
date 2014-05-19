# Find the BOOST Library
FIND_PACKAGE(Boost COMPONENTS system thread REQUIRED)

# Find the NatNet Library
FIND_PACKAGE(NatNet REQUIRED)

# Build our NUClear module
NUCLEAR_MODULE(INCLUDES ${Boost_INCLUDE_DIRS} ${NATNET_INCLUDE_DIRS}
               LIBRARIES ${Boost_LIBRARIES})