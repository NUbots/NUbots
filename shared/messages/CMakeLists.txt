# This directory contains our message types and other shared types
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/shared/)

# This gets all of our source messages
FILE(GLOB_RECURSE src "**.cpp" "**.h")

# Get all protocol buffers
FILE(GLOB_RECURSE protobufs "**.proto")

# If we have any protocol buffers
IF(protobufs)

    # We need Protobuf
    FIND_PACKAGE(Protobuf REQUIRED)

    # We need our protocol buffer headers if we are using protocol buffers
    INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})

    # This directory contains our generated protocol buffers
    INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/shared/)

    # Generate the protocol buffers
    PROTOBUF_GENERATE_CPP(proto_src proto_h ${protobufs})

    # Add the protocol buffers in with our source
    SET(src ${src} ${proto_src} ${proto_h})

ENDIF()

SOURCE_GROUP("Protocol Buffers" FILES ${protobufs})
SOURCE_GROUP("Protocol Buffer Generated Files" FILES ${proto_src} ${proto_h})

# Build a library from these files
ADD_LIBRARY(messages ${protobufs} ${src})
