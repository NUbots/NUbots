# Build our NUClear module

find_package(booster_robotics_sdk REQUIRED)

nuclear_module(LIBRARIES booster_robotics_sdk::booster_robotics_sdk)
