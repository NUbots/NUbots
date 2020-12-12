find_package(VisualMesh REQUIRED)

# Build our NUClear module
nuclear_module(INCLUDES ${VisualMesh_INCLUDE_DIRS} LIBRARIES ${VisualMesh_LIBRARIES})
