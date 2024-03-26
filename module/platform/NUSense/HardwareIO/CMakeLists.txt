# Check subcontroller before we build
if(NOT SUBCONTROLLER STREQUAL "NUSense")
  return()
endif()

# Build our NUClear module
nuclear_module()
