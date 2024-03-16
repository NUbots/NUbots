# Check subcontroller before we build
if(NOT SUBCONTROLLER STREQUAL "NUsense")
    return()
endif()

# Build our NUClear module
nuclear_module()
