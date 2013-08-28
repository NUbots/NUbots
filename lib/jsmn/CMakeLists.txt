if(JSMN_INCLUDE_DIRS)
    # Already found
    set(JSMN_FOUND TRUE)
else (JSMN_INCLUDE_DIRS)
    add_library(jsmn src/jsmn.c)

    set(JSMN_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/ PARENT_SCOPE)
    set(JSMN_LIBRARIES jsmn PARENT_SCOPE)

    mark_as_advanced(JSMN_INCLUDE JSMN_LIBRARIES)
endif(JSMN_INCLUDE_DIRS)
