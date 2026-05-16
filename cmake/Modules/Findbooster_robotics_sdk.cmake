include(ToolchainLibraryFinder)
ToolchainLibraryFinder(
  NAME booster_robotics_sdk
  HEADER booster/robot/b1/b1_api_const.hpp booster/robot/b1/b1_loco_client.hpp
  LIBRARY booster_robotics_sdk
)

# libbooster_robotics_sdk.a is static and has transitive dependencies on Fast-RTPS
find_library(fastrtps_LIBRARY NAMES fastrtps)
find_library(fastcdr_LIBRARY NAMES fastcdr)
if(fastrtps_LIBRARY AND TARGET booster_robotics_sdk::booster_robotics_sdk)
  set_target_properties(booster_robotics_sdk::booster_robotics_sdk
    PROPERTIES INTERFACE_LINK_LIBRARIES "${fastrtps_LIBRARY};${fastcdr_LIBRARY}"
  )
endif()
