#!/bin/bash

ret=0

# Loop through all c/cpp files
while IFS= read -r -d $'\0' line; do

    # Get what our formatted code should be
    fmt=$( clang-format-6.0 -style=file $line )

    # Print a status message so we know what it's doing
    echo "Validating formatting for $line"

    # Check if our text is formatted incorrectly
    if ! cmp -s $line <(echo "$fmt"); then

        # Flag that it is wrong and print the difference
        ret=1
        echo "$line is incorrectly formatted"
        colordiff -u $line <(echo "$fmt")
    fi
# This must be at the bottom since otherwise piping into the while will make a subshell
done < <(find . -type f \( -name *.h \
                        -o -name *.c \
                        -o -name *.cc \
                        -o -name *.cxx \
                        -o -name *.cpp \
                        -o -name *.hpp \
                        -o -name *.ipp \
                        -o -name *.proto \) -print0)

# If we failed somewhere this will exit 1 and fail the build
exit $ret
