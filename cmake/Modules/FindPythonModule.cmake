# Find if a Python module is installed
# Found at http://www.cmake.org/pipermail/cmake/2011-January/041666.html
# To use do: find_python_module(PyQt4 REQUIRED)
FUNCTION(find_python_module module)
    STRING(TOUPPER ${module} module_upper)

    IF(NOT PY_${module_upper})
        IF(ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED")
            SET(${module}_FIND_REQUIRED TRUE)
        ENDIF()

        # Find the system python module path.
        EXECUTE_PROCESS( COMMAND "${PYTHON_EXECUTABLE}" "-c"
            "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
            OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        # A module's location is usually a directory, but for binary modules it's a .so file.
        SET(ENV{PYTHONPATH} "${PYTHONPATH}:${PYTHON_SITE_PACKAGES}")
        SET(ENV{LD_LIBRARY_PATH} "/nubots/toolchain/${PLATFORM}/lib:/nubots/toolchain/lib")
        EXECUTE_PROCESS(COMMAND
            "${PYTHON_EXECUTABLE}" "-c"
            "import re, ${module}; print(re.compile('/__init__.py.*').sub('', ${module}.__file__))"
            RESULT_VARIABLE _${module}_status
            OUTPUT_VARIABLE _${module}_location
            ERROR_QUIET
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        IF(NOT _${module}_status)
            SET("PY_${module_upper}" "${_${module}_location}" CACHE STRING "Location of Python module ${module}" FORCE)
        ENDIF(NOT _${module}_status)
    ENDIF(NOT PY_${module_upper})

    FIND_PACKAGE_HANDLE_STANDARD_ARGS(PY_${module} DEFAULT_MSG PY_${module_upper})
ENDFUNCTION(find_python_module)
