include(ToolchainLibraryFinder)
ToolchainLibraryFinder(
  NAME glib2
  HEADER glib.h
  LIBRARY glib-2.0
  PATH_SUFFIX glib-2.0
)

find_library(glib2-gio_LIBRARY gio-2.0 DOC "The glib-2.0 (glib2-gio) library")

find_library(glib2-gmodule_LIBRARY gmodule-2.0 DOC "The glib-2.0 (glib2-gmodule) library")

find_library(glib2-gobject_LIBRARY gobject-2.0 DOC "The glib-2.0 (glib2-gobject) library")

find_library(glib2-gthread_LIBRARY gthread-2.0 DOC "The glib-2.0 (glib2-gthread) library")

list(
  APPEND glib2_LIBRARIES ${glib2_LIBRARY} ${glib2-gio_LIBRARY} ${glib2-gmodule_LIBRARY} ${glib2-gobject_LIBRARY}
         ${glib2-gthread_LIBRARY}
)
mark_as_advanced(
  glib2_LIBRARIES glib2_LIBRARY glib2-gio_LIBRARY glib2-gmodule_LIBRARY glib2-gobject_LIBRARY glib2-gthread_LIBRARY
)
