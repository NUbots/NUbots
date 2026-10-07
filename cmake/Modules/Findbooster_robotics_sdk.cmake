find_path(
  booster_robotics_sdk_INCLUDE_DIR
  NAMES booster/robot/b1/b1_api_const.hpp booster/robot/b1/b1_loco_client.hpp
  DOC "The booster_robotics_sdk include directory"
)
find_library(
  booster_robotics_sdk_LIBRARY
  NAMES booster_robotics_sdk
  DOC "The booster_robotics_sdk library"
)
mark_as_advanced(booster_robotics_sdk_INCLUDE_DIR booster_robotics_sdk_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  booster_robotics_sdk REQUIRED_VARS booster_robotics_sdk_LIBRARY booster_robotics_sdk_INCLUDE_DIR
)

if(booster_robotics_sdk_FOUND AND NOT TARGET booster_robotics_sdk::booster_robotics_sdk)
  # The SDK ships as a static archive with its own copy of FastDDS inside. Linked straight into each module that uses
  # it (HardwareIO, K1Sensors, K1Camera, ...), every module's shared library carries the SDK's static objects and
  # registers their destructors. Symbol interposition makes them all one object (which is what lets the modules share
  # ChannelFactory::Instance()), so at exit that object is destroyed once per module and the role aborts with "double
  # free or corruption" or "malloc(): unaligned tcache chunk detected". Wrapping the archive in one shared library gives
  # every module the same single copy, constructed and destroyed once.
  if(booster_robotics_sdk_LIBRARY MATCHES "\\.a$")
    if(NOT TARGET booster_robotics_sdk_shared)
      # A shared library needs a source file, the archive supplies everything else. The archive holds Kick.cpp.o twice
      # (byte-identical copies), which only --whole-archive pulls in together, so keep the first definition.
      file(GENERATE OUTPUT "${PROJECT_BINARY_DIR}/booster_robotics_sdk_shared.cpp" CONTENT "")
      add_library(booster_robotics_sdk_shared SHARED "${PROJECT_BINARY_DIR}/booster_robotics_sdk_shared.cpp")
      target_link_libraries(
        booster_robotics_sdk_shared PRIVATE -Wl,--whole-archive "${booster_robotics_sdk_LIBRARY}"
                                            -Wl,--no-whole-archive -Wl,--allow-multiple-definition
      )
      set_property(TARGET booster_robotics_sdk_shared PROPERTY LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin/lib")
    endif()
    set(booster_robotics_sdk_LIBRARIES booster_robotics_sdk_shared)
  else()
    set(booster_robotics_sdk_LIBRARIES "${booster_robotics_sdk_LIBRARY}")
  endif()
  set(booster_robotics_sdk_INCLUDE_DIRS "${booster_robotics_sdk_INCLUDE_DIR}")

  add_library(booster_robotics_sdk::booster_robotics_sdk INTERFACE IMPORTED)
  target_link_libraries(booster_robotics_sdk::booster_robotics_sdk INTERFACE ${booster_robotics_sdk_LIBRARIES})
  target_include_directories(booster_robotics_sdk::booster_robotics_sdk SYSTEM INTERFACE ${booster_robotics_sdk_INCLUDE_DIRS})
endif()
