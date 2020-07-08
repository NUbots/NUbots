# We need python!
find_package(PythonInterp REQUIRED)

# An option for what format of graph we want
set(GRAPH_FILE_TYPE
    svgz
    CACHE STRING "Choose the file format from graphs (e.g. svg, pdf, png)"
)
# We need a demangler that doesn't suck (compiled with the same code as us)
add_library(cxx_demangle SHARED ${CMAKE_CURRENT_SOURCE_DIR}/cxx_demangle.cpp)
set_property(TARGET cxx_demangle PROPERTY LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")

# For all the modules run our information extraction script
foreach(module ${NUCLEAR_MODULES})

  # Get our targets name
  string(REPLACE "::" "" module_name ${module})

  # Add our command to build our json output for the information from the file
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/nuclear_info/${module_name}.json
    COMMAND
      ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/extract_nuclear_info.py ${module} $<TARGET_FILE:${module_name}>
      ${CMAKE_CURRENT_BINARY_DIR}/nuclear_info/${module_name}.json $<TARGET_FILE:cxx_demangle>
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/extract_nuclear_info.py ${module_name} cxx_demangle
    COMMENT "Extracting the NUClear information from ${module}"
  )

endforeach(module)

unset(graph_files)

# For all the roles run our graph generation script
foreach(role ${NUCLEAR_ROLES})

  # Get the modules used by this role
  get_property(
    role_modules
    TARGET ${role}
    PROPERTY NUCLEAR_MODULES
  )

  # Make a list of all the JSON files used for this module
  unset(module_graph_sources)
  foreach(module ${role_modules})
    string(REPLACE "::" "" module_name ${module})
    set(module_graph_sources ${module_graph_sources} ${CMAKE_CURRENT_BINARY_DIR}/nuclear_info/${module_name}.json)
  endforeach(module)

  # Build our .dot files from the json files
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_reaction.dot
           ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_module.dot
           ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_info.json
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/build_role_graph.py
            ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role} ${module_graph_sources}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/build_role_graph.py ${module_graph_sources}
    COMMENT "Generating the emit graphs for the role ${role}"
  )

  # Build .pdf files from our .dot files
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_reaction.${GRAPH_FILE_TYPE}
    COMMAND /usr/bin/dot -T${GRAPH_FILE_TYPE} ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_reaction.dot -o
            ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_reaction.${GRAPH_FILE_TYPE}
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_reaction.dot
    COMMENT "Generating the ${GRAPH_FILE_TYPE} emit graph for the role ${role}"
  )

  # Build .pdf files from our .dot files
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_module.${GRAPH_FILE_TYPE}
    COMMAND /usr/bin/dot -T${GRAPH_FILE_TYPE} ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_module.dot -o
            ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_module.${GRAPH_FILE_TYPE}
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_module.dot
    COMMENT "Generating the ${GRAPH_FILE_TYPE} emit graph for the role ${role}"
  )

  set(graph_files ${graph_files} ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_module.${GRAPH_FILE_TYPE})
  set(graph_files ${graph_files} ${CMAKE_CURRENT_BINARY_DIR}/role_graph/${role}_reaction.${GRAPH_FILE_TYPE})

endforeach(role)

add_custom_target(graphs DEPENDS ${graph_files})
set_property(TARGET graphs PROPERTY EXCLUDE_FROM_ALL TRUE)
set_property(TARGET graphs PROPERTY EXCLUDE_FROM_DEFAULT_BUILD TRUE)
