# Add the VisualMesh project to be built
ADD_SUBDIRECTORY(VisualMesh)

# Build our NUClear module
NUCLEAR_MODULE(INCLUDES}
               LIBRARIES libvisualmesh)
