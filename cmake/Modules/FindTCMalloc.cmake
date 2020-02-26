include(ToolchainLibraryFinder)
ToolchainLibraryFinder(
  NAME tcmalloc
  HEADER gperftools/tcmalloc.h google/tcmalloc.h
  LIBRARY tcmalloc_minimal tcmalloc
)
