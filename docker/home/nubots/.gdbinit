# Create an alias to allow resetting of the terminal from inside of GDB
# This will allow you to effectively disable (n)curses when debugging 
# (n)curses applications
define reset
    shell reset
end
