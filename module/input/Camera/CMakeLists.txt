# Find Aravis
find_package(Aravis REQUIRED)

# Build our NUClear module
nuclear_module(INCLUDES ${Aravis_INCLUDE_DIRS} LIBRARIES ${Aravis_LIBRARIES})
