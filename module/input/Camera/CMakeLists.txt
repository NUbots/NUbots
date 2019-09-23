# Find Aravis
FIND_PACKAGE(Aravis REQUIRED)

# Build our NUClear module
NUCLEAR_MODULE(INCLUDES ${Aravis_INCLUDE_DIRS} 
               LIBRARIES ${Aravis_LIBRARIES})
