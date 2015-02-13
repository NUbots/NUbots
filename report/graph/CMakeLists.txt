# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

# We need a demangler that doesn't suck (compiled with the same code as us)
ADD_LIBRARY(cxx_demangle SHARED ${CMAKE_CURRENT_SOURCE_DIR}/cxx_demangle.cpp)

# For all the modules run our information extraction script
FOREACH(module ${NUCLEAR_MODULES})

    # Get our targets name
    STRING(REPLACE "::" "" module_name ${module})

    # Add our command to build our json output for the information from the file
    ADD_CUSTOM_COMMAND(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${module_name}.json
        COMMAND ${PYTHON_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/extract_nuclear_info.py
                $<TARGET_FILE:${module_name}>
                ${CMAKE_CURRENT_BINARY_DIR}/${module_name}.json
                $<TARGET_FILE:cxx_demangle>
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/extract_nuclear_info.py
                ${module_name}
                cxx_demangle
        COMMENT "Extracting the NUClear information from ${module}"
    )

ENDFOREACH(module)

# For all the roles run our graph generation script
FOREACH(role ${NUCLEAR_ROLES})

    # Get the modules used by this role
    GET_PROPERTY(role_modules TARGET ${role} PROPERTY NUCLEAR_MODULES)

    # Make a list of all the JSON files used for this module
    UNSET(module_graph_sources)
    FOREACH(module ${role_modules})
        STRING(REPLACE "::" "" module_name ${module})
        SET(module_graph_sources ${module_graph_sources} ${CMAKE_CURRENT_BINARY_DIR}/${module_name}.json)
    ENDFOREACH(module)

    ADD_CUSTOM_COMMAND(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${role}.dot
        COMMAND ${PYTHON_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/build_role_graph.py
                ${CMAKE_CURRENT_BINARY_DIR}/${role}.dot
                ${module_graph_sources}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/build_role_graph.py
                ${module_graph_sources}
        COMMENT "Generating the emit graph for the role ${role}"
    )

ENDFOREACH(role)

ADD_CUSTOM_TARGET(gt1
                    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/robocup.dot)

# TODO find all of our modules targets

# TODO for all of those make a custom command that runs the NUClear information extraction script (makes a .yaml or .json for each one)

# TODO for all of the roles make a custom command that takes the generated .yaml or .json files and makes a graph .yaml or .json file

# TODO for all of those .yaml files (the ones from the roles) take the output and make a .dot file for each one that represents the graph