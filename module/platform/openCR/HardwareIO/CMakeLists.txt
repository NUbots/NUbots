# Check subcontroller before we build
if(NOT SUBCONTROLLER STREQUAL OPENCR)
  return()
endif()

# Build our NUClear module
nuclear_module()
