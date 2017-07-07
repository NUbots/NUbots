#!/bin/bash

# Exit on errors
set -e

# Loop through all c/cpp files
find . -type f \( -name *.h -o -name *.c -o -name *.cc -o -name *.cxx -o -name *.cpp -o -name *.hpp -o -name *.ipp \) -print0 | while IFS= read -r -d $'\0' line; do

    # Get the original and formatted code
    src=$( cat $line )
    fmt=$( clang-format-4.0 -style=file $line )

    # Check if our text is formatted
    if [ "$src" != "$fmt" ]; then
        echo "$line is incorrectly formatted"
        colordiff <(echo "$src") <(echo "$fmt")
        fail="fail"
    fi
done

# If at any point we failed, we failed as a whole
if [ "$fail" == "fail" ]; then
    exit 1
fi
