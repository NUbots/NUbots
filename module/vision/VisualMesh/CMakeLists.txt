FIND_PACKAGE(OpenCL REQUIRED)
FIND_PACKAGE(PythonInterp 3 REQUIRED)


FILE(GLOB cls
     "src/mesh/cl/**.cl")

# Convert our opencl files into header files with their contents
FOREACH(cl ${cls})

    FILE(RELATIVE_PATH cl_path ${CMAKE_CURRENT_SOURCE_DIR} ${cl})

    ADD_CUSTOM_COMMAND(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${cl_path}.hpp"
        COMMAND ${PYTHON_EXECUTABLE}
        ARGS "${CMAKE_CURRENT_SOURCE_DIR}/src/wrap_opencl.py"
             "${cl}"
             "${CMAKE_CURRENT_BINARY_DIR}/${cl_path}.hpp"
        DEPENDS "${cl}"
                "${CMAKE_CURRENT_SOURCE_DIR}/src/wrap_opencl.py"
        COMMENT "Wrapping OpenCL file ${cl_path} in a header"
    )
    LIST(APPEND cl_src "${CMAKE_CURRENT_BINARY_DIR}/${cl_path}.hpp")
ENDFOREACH(cl ${cls})

# Build our NUClear module
NUCLEAR_MODULE(INCLUDES ${OpenCL_INCLUDE_DIRS} "${CMAKE_CURRENT_BINARY_DIR}/src/mesh"
               LIBRARIES ${OpenCL_LIBRARIES}
               SOURCES ${cl_src})
