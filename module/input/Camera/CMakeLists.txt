# Find Aravis
find_package(Aravis REQUIRED)

# Build our NUClear module
nuclear_module(LIBRARIES Aravis::Aravis)
