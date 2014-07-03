# Find the  Library
FIND_PACKAGE( REQUIRED)

# Build our NUClear module
NUCLEAR_MODULE(INCLUDES ${_INCLUDE_DIR}
               LIBRARIES ${_LIBRARY})

