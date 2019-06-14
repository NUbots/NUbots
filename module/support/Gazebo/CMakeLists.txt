# Find ignition-transport4 library
FIND_PACKAGE(ignition-transport4 REQUIRED)

# Build our NUClear module
NUCLEAR_MODULE(LIBRARIES ignition-transport4::ignition-transport4)
