# Get the relative path to our message directory
GET_FILENAME_COMPONENT(message_include_dir "${NUCLEAR_MESSAGE_DIR}/.." ABSOLUTE)
FILE(RELATIVE_PATH message_include_dir ${CMAKE_SOURCE_DIR} ${message_include_dir})

# Get our two include directories for message
SET(message_source_include_dir "${CMAKE_SOURCE_DIR}/${message_include_dir}")
SET(message_binary_include_dir "${CMAKE_BINARY_DIR}/${message_include_dir}")

# Make our message include directories variable
SET(NUCLEAR_MESSAGE_INCLUDE_DIRS
    ${message_source_include_dir}
    ${message_binary_include_dir}
    CACHE INTERNAL "Include directories for the message folder and generated sources")
INCLUDE_DIRECTORIES(${NUCLEAR_MESSAGE_INCLUDE_DIRS})

# Get the relative path to our message directory
FILE(RELATIVE_PATH message_dir ${CMAKE_SOURCE_DIR} ${NUCLEAR_MESSAGE_DIR})

# Get our source and binary directories for message
SET(message_source_dir "${CMAKE_SOURCE_DIR}/${message_dir}")
SET(message_binary_dir "${CMAKE_BINARY_DIR}/${message_dir}")

# Use a recursive glob to get all c++ files in the messages folder
FILE(GLOB_RECURSE src
        "${message_source_dir}/**.cpp"
        "${message_source_dir}/**.cc"
        "${message_source_dir}/**.ipp"
        "${message_source_dir}/**.hpp"
        "${message_source_dir}/**.c"
        "${message_source_dir}/**.h"
)

# Use a recursive glob to get all protocol buffers that exist in the messages folder
FILE(GLOB_RECURSE protobufs "${message_source_dir}/**.proto")

# If we have any protocol buffers we need to build them for use
IF(protobufs)

    # We need Protobuf in order to compile them
    FIND_PACKAGE(Protobuf REQUIRED)

    # We need our protocol buffer headers if we are using protocol buffers
    INCLUDE_DIRECTORIES(SYSTEM ${PROTOBUF_INCLUDE_DIRS})

    # Loop through our protocol buffers
    FOREACH(proto ${protobufs})

        # Get the absolute file, and the file without the extension
        GET_FILENAME_COMPONENT(abs_file ${proto} ABSOLUTE)
        GET_FILENAME_COMPONENT(file_we ${proto} NAME_WE)

        # Calculate the Output Directory
        FILE(RELATIVE_PATH outputpath ${message_source_dir} ${proto})
        GET_FILENAME_COMPONENT(outputpath ${outputpath} PATH)
        SET(outputpath "${message_binary_dir}/${outputpath}")

        # Add the two files we will generate to our output
        LIST(APPEND proto_src "${outputpath}/${file_we}.pb.cc")
        LIST(APPEND proto_h   "${outputpath}/${file_we}.pb.h")

        # Compile the protocol buffers
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
ADD_LIBRARY(nuclear_message message.cpp ${protobufs} ${src})

# Link our additional shared libraries
TARGET_LINK_LIBRARIES(nuclear_message ${NUCLEAR_ADDITIONAL_SHARED_LIBRARIES})

# Link to protobuf if we used it
IF(protobufs)
    TARGET_LINK_LIBRARIES(nuclear_message ${PROTOBUF_LIBRARIES})
ENDIF(protobufs)

# Put it in an IDE group for shared
SET_PROPERTY(TARGET nuclear_message PROPERTY FOLDER "shared/")
