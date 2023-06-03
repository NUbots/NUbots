include(ToolchainLibraryFinder)

find_package(kaldi REQUIRED)
find_package(openfst REQUIRED)

ToolchainLibraryFinder(
  NAME vosk
  HEADER vosk_api.h
  LIBRARY vosk
)

target_link_libraries(vosk::vosk INTERFACE openfst::openfst)
target_link_libraries(vosk::vosk INTERFACE kaldi::kaldi)
