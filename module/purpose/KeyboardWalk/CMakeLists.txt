# Find Curses (we need ncurses)
set(CURSES_NEED_NCURSES TRUE)
set(CURSES_NEED_WIDE TRUE)
find_package(Curses REQUIRED)

# Build our NUClear module
nuclear_module(INCLUDES ${CURSES_INCLUDE_DIR} LIBRARIES ${CURSES_LIBRARIES})
