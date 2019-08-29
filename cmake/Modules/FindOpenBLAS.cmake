include(ToolchainLibraryFinder)
ToolchainLibraryFinder(
  NAME OpenBLAS
  HEADER cblas.h
  LIBRARY openblas
  VERSION_FILE openblas_config.h
  VERSION_REGEX "OPENBLAS_VERSION \" OpenBLAS (([0-9]+\\.?)+) \""
)

# TODO also find LAPACK if needed
