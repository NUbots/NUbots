# Find libjpeg library
find_package(JPEG REQUIRED)

find_package(Aravis REQUIRED)
find_package(glib2 REQUIRED)

# Build our NUClear module
nuclear_module(
  INCLUDES ${JPEG_INCLUDE_DIR} ${Aravis_INCLUDE_DIRS} ${glib2_INCLUDE_DIRS}
  LIBRARIES ${JPEG_LIBRARIES} ${Aravis_LIBRARY} ${glib2_LIBRARIES}
)
