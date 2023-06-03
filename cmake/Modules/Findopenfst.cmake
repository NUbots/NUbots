include(ToolchainLibraryFinder)

ToolchainLibraryFinder(
  NAME openfst
  # Not using .h files so not included
  LIBRARIES fst fstfar fstlookahead fstngram
)
