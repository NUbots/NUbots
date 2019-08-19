# List subdirectories
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

# Collect CXX files
macro(GET_CXX_FILES file_list dir)
  file(
    GLOB
    file_list
    "${dir}*.cpp"
    "${dir}*.c"
    "${dir}/*.cpp"
    "${dir}/*.c"
  )
endmacro(GET_CXX_FILES)

# Collect CXX files recursively
macro(GET_CXX_FILES_RECURSE file_list dir)
  file(
    GLOB_RECURSE
    file_list
    "${dir}*.cpp"
    "${dir}*.c"
    "${dir}/*.cpp"
    "${dir}/*.c"
  )
endmacro(GET_CXX_FILES_RECURSE)

# Check if the dependencies of the folder have been found
macro(CHECK_DEPENDENCIES missing)
  unset(${missing})
  set(dependencies ${ARGN})
  foreach(dep ${dependencies})
    string(TOUPPER ${dep} updep)
    if(NOT ((${dep}_FOUND) OR (${updep}_FOUND)))
      list(APPEND ${missing} ${dep})
    endif()
  endforeach()
endmacro(CHECK_DEPENDENCIES)

# Add library if we have the dependencies
macro(ADD_UTILITY_LIBRARY directory)

  # Get missing dependencies
  set(dependencies ${ARGN})
  check_dependencies(missing_dependencies ${dependencies})

  # Generate library name based on path
  file(RELATIVE_PATH current_utility ${NUTILITIES_DIR} ${directory})
  set(current_library nutilities-${current_utility})

  if(NOT missing_dependencies)
    # Get file list: if we have no cxx files we dont need to link
    message(STATUS "Building Utility Library ${current_library}")
    get_cxx_files_recurse(file_list ${directory})
    if(file_list)
      # ADD_LIBRARY(${current_library} ${file_list}) SET(NUTILITIES_LIBRARIES ${NUTILITIES_LIBRARIES} ${current_library}
      # CACHE INTERNAL "Libraries for NUtilities" FORCE)
      set(
        NUTILITIES_LIBRARIES_FILES
        ${NUTILITIES_LIBRARIES_FILES} ${file_list}
        CACHE INTERNAL "Libraries for NUtilities" FORCE
      )
    else()
      # MESSAGE("")
    endif()
  else()
    message(STATUS "NOT Building ${current_library}. Missing dependencies: " ${missing_dependencies})
  endif()

endmacro(ADD_UTILITY_LIBRARY)
