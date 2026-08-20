# Build our NUClear module
find_package(mujoco REQUIRED)
find_package(X11)

nuclear_module(LIBRARIES X11)
