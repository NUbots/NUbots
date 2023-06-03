include(ToolchainLibraryFinder)

find_package(OpenBLAS REQUIRED)

ToolchainLibraryFinder(
  NAME kaldi
  # Not using .h files so not included
  LIBRARIES kaldi-base kaldi-chain kaldi-cudamatrix kaldi-feat
  kaldi-fstext kaldi-gmm kaldi-hmm kaldi-ivector kaldi-kws kaldi-lat kaldi-lm kaldi-matrix
  kaldi-nnet2 kaldi-nnet3 kaldi-nnet kaldi-online2 kaldi-online kaldi-rnnlm kaldi-transform
  kaldi-tree kaldi-util
  )

  target_link_libraries(kaldi::kaldi INTERFACE OpenBLAS::OpenBLAS)
