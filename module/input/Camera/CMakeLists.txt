# Find libjpeg library
FIND_PACKAGE(JPEG REQUIRED)

# Find Spinnaker library
FIND_PACKAGE(Spinnaker REQUIRED)

# Build our NUClear module
NUCLEAR_MODULE(INCLUDES ${JPEG_INCLUDE_DIR} ${Spinnaker_INCLUDE_DIRS}
               LIBRARIES ${JPEG_LIBRARIES} ${Spinnaker_LIBRARY})
