find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(X11 REQUIRED)

# Build our NUClear module
nuclear_module(LIBRARIES mujoco::mujoco glfw X11::X11)
