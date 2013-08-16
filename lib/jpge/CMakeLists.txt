if(JPGE_INCLUDE_DIRS)
    # Already found
    set(JPGE_FOUND TRUE)
else (JPGE_INCLUDE_DIRS)
    add_library(jpge src/jpge.cpp)

    set(JPGE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/ PARENT_SCOPE)
    set(JPGE_LIBRARIES jpge PARENT_SCOPE)

    mark_as_advanced(JPGE_INCLUDE JPGE_LIBRARIES)
endif(JPGE_INCLUDE_DIRS)
