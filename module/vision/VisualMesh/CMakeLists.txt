find_package(OpenCL REQUIRED)
find_package(VisualMesh REQUIRED)

# Build our NUClear module
nuclear_module(LIBRARIES visualmesh::visualmesh)
