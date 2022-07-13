include(ToolchainLibraryFinder)

find_package(libmount REQUIRED)

ToolchainLibraryFinder(
  NAME glib2
  HEADER glib.h
         LIBRARIES
         glib-2.0
         gio-2.0
         gmodule-2.0
         gobject-2.0
         gthread-2.0
  PATH_SUFFIX glib-2.0
  VERSION_FILE "../glibconfig.h"
  VERSION_REGEX "#define GLIB_MAJOR_VERSION ([0-9]+)" "#define GLIB_MINOR_VERSION ([0-9]+)"
                "#define GLIB_MICRO_VERSION ([0-9]+)"
)

# Glib has some internal dependencies between its libraries
target_link_libraries(glib2::gobject-2.0 INTERFACE glib2::glib-2.0)
target_link_libraries(glib2::glib2 INTERFACE libmount::libmount)
