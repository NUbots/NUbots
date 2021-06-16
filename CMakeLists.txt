cmake_minimum_required(VERSION 3.19)
project(NUbots)

# ~~~
# This is needed so we can link in the custom clock object library to the message and utility libraries
# CMP0079: target_link_libraries() allows use with targets in other directories
# https://cmake.org/cmake/help/v3.16/policy/CMP0079.html
# ~~~
if(POLICY CMP0079)
  cmake_policy(SET CMP0079 NEW)
endif()

# Enable testing if we are building tests
if(BUILD_TESTS)
  enable_testing()
endif()

# Set on to prefer .a files over .so files for linking
option(STATIC_LIBRARIES
       "Prefer using static .a files rather than .so files. NOTE: This must be enabled from an empty build directory!"
       OFF
)

if(STATIC_LIBRARIES)
  set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_SUFFIX})
endif()

# We use additional modules that cmake needs to know about
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" "${PROJECT_SOURCE_DIR}/nuclear/cmake/Modules/")

# If this option is set we are building using continous integration
option(CI_BUILD "Enable build options for building in the CI server" OFF)

# Default to do a debug build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE
      Debug
      CACHE STRING "Choose the type of build." FORCE
  )
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Default not to run the clang-tidy checks, default to whatever our CI_BUILD is
option(ENABLE_CLANG_TIDY "Enable building with clang-tidy checks." OFF)
if(ENABLE_CLANG_TIDY OR CI_BUILD)
  find_package(PythonInterp 3 REQUIRED)
  set(CMAKE_CXX_CLANG_TIDY "${PYTHON_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/cmake/scripts/clang-tidy.py"
                           "${PROJECT_BINARY_DIR}/clang-tidy-fixes" clang-tidy
  )
  set(CMAKE_C_CLANG_TIDY ${CMAKE_CXX_CLANG_TIDY})

  # Create a target that will apply clang-tidy fixes to the codebase
  add_custom_target(
    apply-clang-tidy
    COMMAND clang-apply-replacements --format --style=file --style-config="${PROJECT_SOURCE_DIR}"
            --remove-change-desc-files "${PROJECT_BINARY_DIR}/clang-tidy-fixes"
    COMMENT "Applying fixes from clang-tidy to the codebase."
  )
endif()

# If we are doing a CI build then we want to enable -Werror when compiling warnings are bad. We will also make it fail
# if clang-tidy has an error
if(CI_BUILD)
  add_compile_options(-Werror)
  set(CMAKE_CXX_CLANG_TIDY ${CMAKE_CXX_CLANG_TIDY} -warnings-as-errors=*)
  set(CMAKE_C_CLANG_TIDY ${CMAKE_C_CLANG_TIDY} -warnings-as-errors=*)
endif()

# Options common to all sanitizers. See https://github.com/google/sanitizers/wiki/SanitizerCommonFlags
# TODO(KipHamiltons): add python code to run.py to print an advisory and set the env vars that way
set(SANITIZER_OPTIONS
    "fast_unwind_on_check=true fast_unwind_on_fatal=true external_symbolizer_path=/usr/local/bin/llvm-symbolizer"
)
# Verbosity of sanitizers gets its own variable, so we can set it manually
set(VERBOSE_SANITIZERS
    "0"
    CACHE STRING "Verbosity of Sanitizers. # 0 (default) => silent, 1 => a bit of output, 2 => more output."
)
if(VERBOSE_SANITIZERS STREQUAL "2")
  string(APPEND SANITIZER_OPTIONS " verbosity=2")
elseif(VERBOSE_SANITIZERS STREQUAL "1")
  string(APPEND SANITIZER_OPTIONS " verbosity=1")
endif()

# Enable address sanitizer (implicitly enables leak sanitizer too)
option(USE_ASAN "Enable address sanitization" OFF)
if(USE_ASAN)
  # These sanitizers are mutually exclusive
  if(USE_MSAN)
    message(FATAL_ERROR "You have ADDRESS sanitizer and MEMORY sanitizer on. These are incompatible together"
                        "Turn one of them off to get rid of this error."
    )
  elseif(USE_TSAN)
    message(FATAL_ERROR "You have ADDRESS sanitizer and THREAD sanitizer on. These are incompatible together"
                        "Turn one of them off to get rid of this error."
    )
  endif()
  # Set the variables/flags for running ASAN
  add_compile_options(-fsanitize=address -g -fno-omit-frame-pointer -U_FORTIFY_SOURCE -fno-common)
  add_link_options(-fsanitize=address)
  link_libraries(asan)
  set(ENV{ASAN_OPTIONS} SANITIZER_OPTIONS)
endif()

# Enable thread sanitizer
option(USE_TSAN "Enable thread sanitization. Note that there is an enormous expected overhead" OFF)
if(USE_TSAN)
  # These sanitizers are mutually exclusive
  if(USE_MSAN)
    message(FATAL_ERROR "You have THREAD sanitizer and MEMORY sanitizer on. These are incompatible together"
                        "Turn one of them off to get rid of this error."
    )
  endif()
  # We need to use clang to use TSAN
  message(STATUS "Memory sanitizer enabled. Forcing clang as project compiler")
  set(PROJECT_COMPILER
      "clang"
      CACHE STRING "Compiler to use for C and C++ source."
  )
  # Set the variables/flags for running TSAN
  add_compile_options(-fsanitize=thread -g -fno-omit-frame-pointer -U_FORTIFY_SOURCE -fno-common)
  add_link_options(-fsanitize=thread)
  link_libraries(tsan)
  set(ENV{TSAN_OPTIONS} SANITIZER_OPTIONS)
endif()

# Enable memory sanitizer
option(USE_MSAN "Enable memory sanitization" OFF)
if(USE_MSAN)
  # Memory sanitizer does not work with static libraries. We exit if both are enabled
  if(STATIC_LIBRARIES)
    message(FATAL_ERROR "You have memory sanitizer and static libraries on. These are incompatible together."
                        "Turn one of them off to get rid of this error."
    )
  endif()
  # We need to use clang to use MSAN
  message(STATUS "Memory sanitizer enabled. Forcing clang as project compiler")
  set(PROJECT_COMPILER
      "clang"
      CACHE STRING "Compiler to use for C and C++ source."
  )
  # Set the variables/flags for running MSAN
  add_compile_options(
    -fsanitize=memory -g -fno-omit-frame-pointer -U_FORTIFY_SOURCE -fno-common -fsanitize-memory-use-after-dtor
  )
  add_link_options(-fsanitize=memory)
  set(ENV{MSAN_OPTIONS} SANITIZER_OPTIONS)
endif()

# Enable undefined behaviour sanitizer
option(USE_UBSAN "Enable undefined behaviour sanitization" OFF)
if(USE_UBSAN)
  # -fsanitize has several extra options for undefined or bug-prone behaviour we can enable as well, so we add them too
  add_compile_options(
    -fsanitize=undefined,float-divide-by-zero,unsigned-integer-overflow,implicit-conversion,local-bounds,nullability -g
    -fno-omit-frame-pointer -U_FORTIFY_SOURCE -fno-common -fno-sanitize-recover=1
  )
  add_link_options(-fsanitize=undefined)
  link_libraries(ubsan)
  # Set the variables/flags for running UBSAN
  set(UBSAN_OPTIONS SANITIZER_OPTIONS)
  string(APPEND UBSAN_OPTIONS " UBSAN_OPTIONS=print_stacktrace=1")
  set(ENV{UBSAN_OPTIONS} UBSAN_OPTIONS)
endif()

# Set the compiler for the project
if(PROJECT_COMPILER STREQUAL "clang")
  set(CMAKE_C_COMPILER "clang")
  set(CMAKE_CXX_COMPILER "clang")
elseif(PROJECT_COMPILER STREQUAL "gcc")
  set(CMAKE_C_COMPILER "gcc")
  set(CMAKE_CXX_COMPILER "gcc")
elseif(PROJECT_COMPILER)
  message(WARNING "Unsupported compiler selected. Defaulting to gcc")
  set(PROJECT_COMPILER
      "gcc"
      CACHE STRING "Compiler to use for C and C++ source."
  )
  set(CMAKE_C_COMPILER "gcc")
  set(CMAKE_CXX_COMPILER "gcc")
else()
  # Don't bother warning if no compiler was selected. We default to using gcc
  set(PROJECT_COMPILER
      "gcc"
      CACHE STRING "Compiler to use for C and C++ source."
  )
  set(CMAKE_C_COMPILER "gcc")
  set(CMAKE_CXX_COMPILER "gcc")
endif()

# Set the possible compilers we use for cmake-gui
set_property(CACHE PROJECT_COMPILER PROPERTY STRINGS "clang" "gcc")

# Output the compilation database
set(CMAKE_EXPORT_COMPILE_COMMANDS
    ON
    CACHE STRING "Enable/Disable output of compile commands during generation." FORCE
)

# Enable all warnings that we can
add_compile_options(-Wall -Wpedantic -Wextra)

# This is needed everywhere so that we can catch segmentation faults
add_compile_options(-fnon-call-exceptions)

# Make NUClear use the system clock, as we are syncing it with ptp
add_compile_definitions(NUCLEAR_CLOCK_TYPE=std::chrono::steady_clock)

# Make the compiler display colours always (even when we build with ninja)
if(CMAKE_CXX_COMPILER_ID MATCHES GNU)
  add_compile_options(-fdiagnostics-color=always)
elseif(CMAKE_CXX_COMPILER_ID MATCHES Clang)
  add_compile_options(-fcolor-diagnostics)
endif()

# Our banner file for placing at the top of the roles
set(NUCLEAR_ROLE_BANNER_FILE
    "${PROJECT_SOURCE_DIR}/cmake/banner.png"
    CACHE PATH "The path the banner to print at the start of each role execution" FORCE
)

# Include the NUClear roles system
add_subdirectory(nuclear)

# Setting to use the custom clock with an adaptable rate
option(USE_CUSTOM_CLOCK "Replace NUClear's clock with a custom implementation." ON)
if(USE_CUSTOM_CLOCK)
  add_subdirectory(shared/clock)
endif()

# Add the shared tests. Must happen after adding NUClear so that this variable is cached
if(BUILD_TESTS)
  add_subdirectory(${NUCLEAR_SHARED_DIR}/tests)
endif()
