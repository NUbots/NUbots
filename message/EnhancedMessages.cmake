# Get the relative path to our message directory
GET_FILENAME_COMPONENT(message_include_dir "${NUCLEAR_MESSAGE_DIR}/.." ABSOLUTE)
FILE(RELATIVE_PATH message_include_dir ${CMAKE_SOURCE_DIR} ${message_include_dir})

# Get our two include directories for message
SET(message_source_include_dir "${CMAKE_SOURCE_DIR}/${message_include_dir}")
SET(message_binary_include_dir "${CMAKE_BINARY_DIR}/${message_include_dir}")

# Make our message include directories variable
SET(NUCLEAR_MESSAGE_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${message_source_include_dir}
    ${message_binary_include_dir}
    CACHE INTERNAL "Include directories for the message folder and generated sources")
INCLUDE_DIRECTORIES(${NUCLEAR_MESSAGE_INCLUDE_DIRS})

# Get the relative path to our message directory
FILE(RELATIVE_PATH message_dir ${CMAKE_SOURCE_DIR} ${NUCLEAR_MESSAGE_DIR})

# Get our source and binary directories for message
SET(message_source_dir "${CMAKE_SOURCE_DIR}/${message_dir}")
SET(message_binary_dir "${CMAKE_BINARY_DIR}/${message_dir}")

# We need protobuf and python to generate the enhanced messages
FIND_PACKAGE(Protobuf REQUIRED)
FIND_PACKAGE(PythonInterp REQUIRED)

# Build our builtin protobuf classes
FILE(GLOB_RECURSE builtin "${CMAKE_CURRENT_SOURCE_DIR}/proto/**.proto")
FOREACH(proto ${builtin})

    # Get the file without the extension
    GET_FILENAME_COMPONENT(file_we ${proto} NAME_WE)

    # Run the protocol buffer compiler on the builtin protocol buffers
    ADD_CUSTOM_COMMAND(
        OUTPUT "${message_binary_include_dir}/${file_we}.pb.cc"
               "${message_binary_include_dir}/${file_we}.pb.h"
               "${message_binary_include_dir}/${file_we}_pb2.py"
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --cpp_out=lite:${message_binary_include_dir}
             --python_out=${message_binary_include_dir}
             -I${CMAKE_CURRENT_SOURCE_DIR}/proto
             "${CMAKE_CURRENT_SOURCE_DIR}/proto/${file_we}.proto"
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/proto/${file_we}.proto"
        COMMENT "Compiling protocol buffer ${proto}")

    SET(src ${src}
            "${message_binary_include_dir}/${file_we}.pb.cc"
            "${message_binary_include_dir}/${file_we}.pb.h"
            "${message_binary_include_dir}/${file_we}_pb2.py")

ENDFOREACH(proto)

# Get our dependency files for our message class generator
FILE(GLOB_RECURSE message_class_generator_depends "${CMAKE_CURRENT_SOURCE_DIR}/generator/**.py")

# Build all of our normal messages
FILE(GLOB_RECURSE protobufs "${message_source_dir}/**.proto")
FOREACH(proto ${protobufs})

    # Get the file without the extension
    GET_FILENAME_COMPONENT(file_we ${proto} NAME_WE)

    # Calculate the Output Directory
    FILE(RELATIVE_PATH outputpath ${message_source_dir} ${proto})
    GET_FILENAME_COMPONENT(outputpath ${outputpath} PATH)
    SET(outputpath "${message_binary_dir}/${outputpath}")

    # Create the output directory
    FILE(MAKE_DIRECTORY ${outputpath})

    # Get the dependencies on this protobuf so we can recompile on changes
    # If they change you will need to re cmake
    EXECUTE_PROCESS(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
                            --dependency_out=${CMAKE_CURRENT_BINARY_DIR}/temp1
                            --descriptor_set_out=${CMAKE_CURRENT_BINARY_DIR}/temp2
                            -I${message_source_include_dir}
                            -I${CMAKE_CURRENT_SOURCE_DIR}/proto
                            ${proto})

    FILE(READ "${CMAKE_CURRENT_BINARY_DIR}/temp1" dependencies)
    STRING(REGEX REPLACE "\\\\\n" ";" dependencies ${dependencies})
    FILE(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/temp1" "${CMAKE_CURRENT_BINARY_DIR}/temp2")

    UNSET(source_depends)
    UNSET(binary_depends)

    # Clean our dependency list
    FOREACH(depend ${dependencies})
        STRING(STRIP ${depend} depend)
        FILE(RELATIVE_PATH depend_rel ${message_source_dir} ${depend})

        # Absolute dependencies
        IF(depend_rel MATCHES "^\\.\\.")
            SET(source_depends ${source_depends} ${depend})
            SET(binary_depends ${binary_depends} ${depend})
        # Relative dependencies
        ELSE()
            SET(source_depends ${source_depends} "${message_source_dir}/${depend_rel}")
            SET(binary_depends ${binary_depends} "${message_binary_dir}/${depend_rel}")
        ENDIF()
    ENDFOREACH()

    # Extract the protocol buffer information so we can generate code off it
    ADD_CUSTOM_COMMAND(
        OUTPUT "${outputpath}/${file_we}.pb"
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --descriptor_set_out="${outputpath}/${file_we}.pb"
             -I${message_source_include_dir}
             -I${CMAKE_CURRENT_SOURCE_DIR}/proto
             ${proto}
        DEPENDS ${source_depends}
        COMMENT "Extracting protocol buffer information from ${proto}")

    # Repackage our protocol buffers so they don't collide with the actual classes
    # when we make our c++ protobuf classes by adding protobuf to the package
    ADD_CUSTOM_COMMAND(
        OUTPUT "${outputpath}/${file_we}.proto"
        COMMAND ${PYTHON_EXECUTABLE}
        ARGS "${CMAKE_CURRENT_SOURCE_DIR}/repackage_message.py" ${proto} ${outputpath}
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/repackage_message.py"
                ${source_depends}
        COMMENT "Repackaging protobuf ${proto}")

    # Run the protocol buffer compiler on these new protobufs
    ADD_CUSTOM_COMMAND(
        OUTPUT "${outputpath}/${file_we}.pb.cc"
               "${outputpath}/${file_we}.pb.h"
               "${outputpath}/${file_we}_pb2.py"
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --cpp_out=lite:${message_binary_include_dir}
             --python_out=${message_binary_include_dir}
             -I${message_binary_include_dir}
             -I${CMAKE_CURRENT_SOURCE_DIR}/proto
             "${outputpath}/${file_we}.proto"
        DEPENDS ${binary_depends}
        COMMENT "Compiling protocol buffer ${proto}")

    # Build our c++ class from the extracted information
    ADD_CUSTOM_COMMAND(
        OUTPUT "${outputpath}/${file_we}.cpp"
               "${outputpath}/${file_we}.h"
        COMMAND ${PYTHON_EXECUTABLE}
        ARGS "${CMAKE_CURRENT_SOURCE_DIR}/build_message_class.py" "${outputpath}/${file_we}" ${outputpath}
        WORKING_DIRECTORY ${message_binary_dir}
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/build_message_class.py"
                ${message_class_generator_depends}
                "${message_binary_include_dir}/MessageOptions_pb2.py"
                "${outputpath}/${file_we}.pb"
        COMMENT "Building classes for ${proto}")

    # The protobuf descriptions are generated
    SET_SOURCE_FILES_PROPERTIES("${outputpath}/${file_we}.pb"
                                "${outputpath}/${file_we}.proto"
                                "${outputpath}/${file_we}.pb.cc"
                                "${outputpath}/${file_we}.pb.h"
                                "${outputpath}/${file_we}.cpp"
                                "${outputpath}/${file_we}.h"
                                 PROPERTIES GENERATED TRUE)

    # Add the generated files to our list
    SET(src ${src}
            "${outputpath}/${file_we}.pb.cc"
            "${outputpath}/${file_we}.pb.h"
            "${outputpath}/${file_we}.cpp"
            "${outputpath}/${file_we}.h")

ENDFOREACH(proto)

IF(src)
    # Build a library from these files
    ADD_LIBRARY(nuclear_message ${protobufs} ${src})

    # The library uses protocol buffers
    TARGET_LINK_LIBRARIES(nuclear_message ${PROTOBUF_LIBRARIES})

    # Add to our list of NUClear message libraries
    SET(NUCLEAR_MESSAGE_LIBRARIES nuclear_message CACHE INTERNAL "List of libraries that are built as messages" FORCE)

    # Put it in an IDE group for shared
    SET_PROPERTY(TARGET nuclear_message PROPERTY FOLDER "shared/")
ENDIF()
