#!/bin/bash

check_formatting() {
    echo "Validating formatting for $1"
    clang-format-6.0 -style=file "$1" | colordiff --color=yes -u "$1" -
    return $?
}
export -f check_formatting

# Loop through all c/cpp/proto files and check validation
git ls-files | grep '.*\.\(c\|cc\|cpp\|cxx\|hpp\|ipp\|proto\)$' \
     | parallel --joblog formatting.log -j$(nproc) check_formatting
# Count how many returned a non zero exist status
ret=$(tail -n +2 formatting.log | awk '{ sum += $7; } END {print sum}')

echo "$ret files are not formatted correctly"
exit $ret
