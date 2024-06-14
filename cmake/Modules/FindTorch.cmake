include(ToolchainLibraryFinder)
ToolchainLibraryFinder(
  NAME torch
  LIBRARY /usr/lib/torch
  HEADER torch/torch.h
  PATHS /usr/include/torch/csrc/api/include/
)
