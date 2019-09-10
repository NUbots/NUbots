# Find libjpeg library
find_package(JPEG REQUIRED)

# Build our NUClear module
nuclear_module(INCLUDES ${JPEG_INCLUDE_DIR} LIBRARIES ${JPEG_LIBRARIES})
