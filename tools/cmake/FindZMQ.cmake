INCLUDE(ToolchainLibraryFinder)
ToolchainLibraryFinder(NAME ZMQ
                       HEADER zmq.h
                       LIBRARY zmq
                       VERSION_FILE zmq.h
                       VERSION_REGEX "ZMQ_VERSION_MAJOR ([0-9]+)"
                                     "ZMQ_VERSION_MINOR ([0-9]+)"
                                     "ZMQ_VERSION_PATCH ([0-9]+)"
)
