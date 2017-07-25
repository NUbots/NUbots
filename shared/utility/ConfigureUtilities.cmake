#List subdirectories
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

#Collect CXX files
MACRO(GET_CXX_FILES file_list dir)
	FILE(GLOB file_list
				"${dir}*.cpp"
				"${dir}*.c"
				"${dir}/*.cpp"
				"${dir}/*.c")
ENDMACRO(GET_CXX_FILES)

#Collect CXX files recursively
MACRO(GET_CXX_FILES_RECURSE file_list dir)
	FILE(GLOB_RECURSE file_list
				"${dir}*.cpp"
				"${dir}*.c"
				"${dir}/*.cpp"
				"${dir}/*.c")
ENDMACRO(GET_CXX_FILES_RECURSE)

#Check if the dependencies of the folder have been found
MACRO(CHECK_DEPENDENCIES missing)
	UNSET(${missing})
	SET(dependencies ${ARGN})
	FOREACH(dep ${dependencies})
		STRING(TOUPPER ${dep} updep)
		IF(NOT ((${dep}_FOUND) OR (${updep}_FOUND)))
			LIST(APPEND ${missing} ${dep})
		ENDIF()
	ENDFOREACH()
ENDMACRO(CHECK_DEPENDENCIES)

#Add library if we have the dependencies
MACRO(ADD_UTILITY_LIBRARY directory)

	#Get missing dependencies
	SET(dependencies ${ARGN})
	CHECK_DEPENDENCIES(missing_dependencies ${dependencies})

	#Generate library name based on path
	FILE(RELATIVE_PATH current_utility ${NUTILITIES_DIR} ${directory})
	SET(current_library nutilities-${current_utility})

	IF(NOT missing_dependencies)
		#Get file list: if we have no cxx files we dont need to link
		MESSAGE(STATUS "Building Utility Library ${current_library}")
		GET_CXX_FILES_RECURSE(file_list ${directory})
		IF(file_list)
			# ADD_LIBRARY(${current_library} ${file_list})
			# SET(NUTILITIES_LIBRARIES ${NUTILITIES_LIBRARIES} ${current_library} CACHE INTERNAL "Libraries for NUtilities" FORCE)
			SET(NUTILITIES_LIBRARIES_FILES ${NUTILITIES_LIBRARIES_FILES} ${file_list} CACHE INTERNAL "Libraries for NUtilities" FORCE)
		ELSE()
			# MESSAGE("")
		ENDIF()
	ELSE()
		MESSAGE(STATUS "NOT Building ${current_library}. Missing dependencies: " ${missing_dependencies})
	ENDIF()

ENDMACRO(ADD_UTILITY_LIBRARY)
