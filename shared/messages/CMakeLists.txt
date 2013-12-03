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

    # Loop through our protocol buffers
    FOREACH(proto ${protobufs})

        # Get the absolute file, and the file without the extension
        GET_FILENAME_COMPONENT(abs_file ${proto} ABSOLUTE)
        GET_FILENAME_COMPONENT(file_we ${proto} NAME_WE)

        # Calculate the Output Directory
        FILE(RELATIVE_PATH outputpath ${CMAKE_CURRENT_SOURCE_DIR} ${proto})
        GET_FILENAME_COMPONENT(outputpath ${outputpath} PATH)
        SET(outputpath "${CMAKE_CURRENT_BINARY_DIR}/${outputpath}")

        # Add the two files we will generate to our output
        LIST(APPEND proto_src "${outputpath}/${file_we}.pb.cc")
        LIST(APPEND proto_h   "${outputpath}/${file_we}.pb.h")

        # Make the protocol buffers
        ADD_CUSTOM_COMMAND(
            OUTPUT "${outputpath}/${file_we}.pb.cc"
                   "${outputpath}/${file_we}.pb.h"
            COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
            ARGS --cpp_out ${CMAKE_BINARY_DIR}/shared -I ${CMAKE_SOURCE_DIR}/shared ${abs_file}
            DEPENDS ${abs_file}
            COMMENT "Running C++ protocol buffer compiler on ${proto}"
            VERBATIM)
    ENDFOREACH()

    # The protobuf files are generated
    SET_SOURCE_FILES_PROPERTIES(${proto_src} ${proto_h} PROPERTIES GENERATED TRUE)

    # Add the protocol buffers in with our source
    SET(src ${src} ${proto_src} ${proto_h})

ENDIF()

SOURCE_GROUP("Protocol Buffers" FILES ${protobufs})
SOURCE_GROUP("Protocol Buffer Generated Files" FILES ${proto_src} ${proto_h})

# Build a library from these files
ADD_LIBRARY(messages ${protobufs} ${src})