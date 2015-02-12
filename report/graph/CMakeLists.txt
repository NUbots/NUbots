# We need python!
FIND_PACKAGE(PythonInterp REQUIRED)

# For all the modules run our information extraction script
FOREACH(module ${NUCLEAR_MODULES})

    # Get our targets name
    STRING(REPLACE "::" "" module_name ${module})
    MESSAGE(${module_name})

    # Add our command to build our json output for the information from the file
    ADD_CUSTOM_COMMAND(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${module_name}.json
        COMMAND ${PYTHON_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/extract_nuclear_info.py
                $<TARGET_FILE:${module_name}>
                ${CMAKE_CURRENT_BINARY_DIR}/${module_name}.json
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/extract_nuclear_info.py
                ${module_name}
        COMMENT "Extracting the NUClear information from ${module}"
    )

ENDFOREACH(module)

ADD_CUSTOM_TARGET(gt1
                    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/supportNUbugger.json)

# TODO find all of our modules targets

# TODO for all of those make a custom command that runs the NUClear information extraction script (makes a .yaml or .json for each one)

# TODO for all of the roles make a custom command that takes the generated .yaml or .json files and makes a graph .yaml or .json file

# TODO for all of those .yaml files (the ones from the roles) take the output and make a .dot file for each one that represents the graph