# Build our NUClear module

find_package(Torch REQUIRED)
target_link_libraries(nuclear_utility PRIVATE "torch;torch_library")
target_include_directories(nuclear_utility PUBLIC ${TORCH_INCLUDE_DIRS})

nuclear_module()
