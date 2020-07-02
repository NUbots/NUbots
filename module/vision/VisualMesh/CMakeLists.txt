# Add the VisualMesh project to be built
add_subdirectory(VisualMesh)

# Build our NUClear module
nuclear_module(INCLUDES} LIBRARIES libvisualmesh)
