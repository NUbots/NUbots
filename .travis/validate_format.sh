#!/bin/bash

ret=0

# Loop through all c/cpp files
find . -type f \( -name *.h -o -name *.c -o -name *.cc -o -name *.cxx -o -name *.cpp -o -name *.hpp -o -name *.ipp \) -print0 | while IFS= read -r -d $'\0' line; do

    # Get the original and formatted code
    src=$( cat $line )
    fmt=$( clang-format-4.0 -style=file $line )

    # Check if our text is formatted
    if [ "$src" != "$fmt" ]; then
        echo "$line is incorrectly formatted"
        if ! colordiff <(echo "$src") <(echo "$fmt"); then
            ret=1
        fi
    fi
done

exit $ret
