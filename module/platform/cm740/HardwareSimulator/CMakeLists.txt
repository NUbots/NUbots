# Check subcontroller before we build
if(NOT SUBCONTROLLER STREQUAL CM740)
  return()
endif()

# Build our NUClear module
nuclear_module()
