INCLUDE(ToolchainLibraryFinder)
ToolchainLibraryFinder(NAME Quex
                       HEADER quex/code_base/analyzer/C-adaptions.h
                       BINARY quex
                       VERSION_BINARY_ARGUMENTS "--version"
                       VERSION_REGEX "Version (([0-9]+\\.?)+)"
)