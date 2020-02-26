# Add the VisualMesh project to be built
add_subdirectory(VisualMesh)

# Set the CL Target Version
add_compile_definitions(CL_TARGET_OPENCL_VERSION=120)

# Build our NUClear module
nuclear_module(INCLUDES} LIBRARIES libvisualmesh)
