# * Comprehensive warnings set
# * Sources:
# * https://github.com/lefticus/cppbestpractices/blob/master/02-Use_the_Tools_Available.md
# * https://github.com/lefticus/cpp_starter_project/blob/master/cmake/CompilerWarnings.cmake

function(set_target_warnings target_name)

  set(TARGET_WARNINGS
      -Wall
      -Wextra # reasonable and standard
      -Wshadow # warn the user if a variable declaration shadows one from a parent context
      -Wnon-virtual-dtor # warn the user if a class with virtual functions has a non-virtual destructor. This helps
                         # catch hard to track down memory errors
      -Wold-style-cast # warn for c-style casts
      -Wcast-align # warn for potential performance problem casts
      -Wunused # warn on anything being unused
      -Woverloaded-virtual # warn if you overload (not override) a virtual function
      -Wpedantic # warn if non-standard C++ is used
      -Wconversion # warn on type conversions that may lose data
      -Wsign-conversion # warn on sign conversions
      -Wdouble-promotion # warn if float is implicit promoted to double
      # -Wformat=2 # warn on security issues around functions that format output (ie printf)
      -Wnull-dereference # warn if a null dereference is detected
      -Wmisleading-indentation # warn if indentation implies blocks where blocks do not exist
      # * We want the warnings below if clang adds support for them
      # * Until then, these make clang-tidy mad, and we would have to have an extra CI pipeline step to cover them
      # * -Wlogical-op # warn about logical operations being used where bitwise were probably wanted
      # * -Wuseless-cast # warn if you perform a cast to the same type
  )

  if(WARNINGS_AS_ERRORS)
    list(APPEND TARGET_WARNINGS -Werror)
  endif()

  target_compile_options(${target_name} PRIVATE ${TARGET_WARNINGS})

endfunction()
