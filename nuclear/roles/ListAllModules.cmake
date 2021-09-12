function(ListAllModules module_dir)

  file(
    GLOB_RECURSE module_cmakelists
    RELATIVE "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}"
    "${PROJECT_SOURCE_DIR}/${NUCLEAR_MODULE_DIR}/**/CMakeLists.txt"
  )

  foreach(module ${module_cmakelists})
    string(REGEX REPLACE "/CMakeLists.txt" "" module ${module})
    string(REGEX REPLACE "/" "::" module ${module})
    list(APPEND module_list ${module})
  endforeach(module ${module_cmakelists})

  set(ALL_MODULES
      ${module_list}
      PARENT_SCOPE
  )
endfunction(ListAllModules)
