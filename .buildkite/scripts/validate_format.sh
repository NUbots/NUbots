#!/bin/sh

ret=0

find . -type f \( -name *.h \
                  -o -name *.c \
                  -o -name *.cc \
                  -o -name *.cxx \
                  -o -name *.cpp \
                  -o -name *.hpp \
                  -o -name *.ipp \
                  -o -name *.proto \) > files.txt

# Loop through all c/cpp files
while IFS= read -r in_file; do

    # Get what our formatted code should be
    clang-format -style=file $in_file > correctly_formatted

    # Print a status message so we know what it's doing
    echo "Validating formatting for $in_file"

    # Check if our text is formatted incorrectly
    if ! cmp -s $in_file correctly_formatted; then

        # Flag that it is wrong and print the difference
        ret=1
        echo "$in_file is incorrectly formatted"
        colordiff -u $in_file correctly_formatted
    fi
# This must be at the bottom since otherwise piping into the while will make a subshell
done < files.txt

# If we failed somewhere this will exit 1 and fail the build
exit $ret
