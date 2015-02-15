# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

# We need a demangler that doesn't suck (compiled with the same code as us)
ADD_LIBRARY(cxx_demangle SHARED ${CMAKE_CURRENT_SOURCE_DIR}/cxx_demangle.cpp)
SET_PROPERTY(TARGET cxx_demangle PROPERTY LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")

# For all the modules run our information extraction script
FOREACH(module ${NUCLEAR_MODULES})

    # Get our targets name
    STRING(REPLACE "::" "" module_name ${module})

    # Add our command to build our json output for the information from the file
    ADD_CUSTOM_COMMAND(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/nuclear_info/${module_name}.json
        COMMAND ${PYTHON_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/extract_nuclear_info.py
                ${module}
                $<TARGET_FILE:${module_name}>
                ${CMAKE_CURRENT_BINARY_DIR}/nuclear_info/${module_name}.json
                $<TARGET_FILE:cxx_demangle>
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/extract_nuclear_info.py
                ${module_name}
                cxx_demangle
        COMMENT "Extracting the NUClear information from ${module}"
    )

ENDFOREACH(module)

UNSET(svg_graphs)

# For all the roles run our graph generation script
FOREACH(role ${NUCLEAR_ROLES})

    # Get the modules used by this role
    GET_PROPERTY(role_modules TARGET ${role} PROPERTY NUCLEAR_MODULES)

    # Make a list of all the JSON files used for this module
    UNSET(module_graph_sources)
    FOREACH(module ${role_modules})
        STRING(REPLACE "::" "" module_name ${module})
        SET(module_graph_sources ${module_graph_sources} ${CMAKE_CURRENT_BINARY_DIR}/nuclear_info/${module_name}.json)
    ENDFOREACH(module)

    # Build our .dot files from the json files
    ADD_CUSTOM_COMMAND(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}.dot
        COMMAND ${PYTHON_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/build_role_graph.py
                ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}.dot
                ${module_graph_sources}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/build_role_graph.py
                ${module_graph_sources}
        COMMENT "Generating the emit graph for the role ${role}"
    )

    # Build .pdf files from our .dot files
    ADD_CUSTOM_COMMAND(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}.svg
        COMMAND /usr/bin/dot
                -Tsvg
                ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}.dot
                -o
                ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}.svg
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}.dot
        COMMENT "Generating the svg emit graph for the role ${role}"
    )

    SET(svg_graphs ${svg_graphs} ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}.svg)

ENDFOREACH(role)

ADD_CUSTOM_TARGET(graphs DEPENDS ${svg_graphs})
