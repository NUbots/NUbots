include(ToolchainLibraryFinder)
ToolchainLibraryFinder(
  NAME tcmalloc
  HEADER gperftools/tcmalloc.h google/tcmalloc.h
  LIBRARY tcmalloc_minimal tcmalloc
  VERSION_FILE gperftools/tcmalloc.h
  VERSION_REGEX
    # define TC_VERSION_MAJOR  2
    # define TC_VERSION_MINOR  4
    # define TC_VERSION_PATCH  ""
    # define TC_VERSION_STRING "gperftools 2.4"
)
