INCLUDE(FindPythonModule)
FIND_PYTHON_MODULE(tensorflow REQUIRED)

INCLUDE(ToolchainLibraryFinder)
ToolchainLibraryFinder(NAME tensorflow
                       HEADER tensorflow/core/framework/tensor.h
)

FIND_LIBRARY (
    tensorflow-cc_LIBRARY
    tensorflow_cc
    DOC "The tensorflow (tensorflow_cc) library"
)

FIND_LIBRARY (
    tensorflow-framework_LIBRARY
    tensorflow_framework
    DOC "The tensorflow (tensorflow_framework) library"
)

LIST(APPEND tensorflow_INCLUDE_DIRS "${tensorflow_INCLUDE_DIRS}/external/nsync/public")

LIST(APPEND tensorflow_LIBRARIES ${tensorflow-cc_LIBRARY} ${tensorflow-framework_LIBRARY})
MARK_AS_ADVANCED(tensorflow_LIBRARIES ${tensorflow-cc_LIBRARY} ${tensorflow-framework_LIBRARY})
