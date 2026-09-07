include(ToolchainLibraryFinder)
ToolchainLibraryFinder(
  NAME booster_robotics_sdk
  HEADER booster/robot/b1/b1_api_const.hpp booster/robot/b1/b1_loco_client.hpp
  LIBRARY booster_robotics_sdk
)
