#!/bin/bash
set -e

check_formatting() {
    echo "Validating formatting for $1"
    clang-format -style=file "$1" | colordiff --color=yes -u "$1" -
    return $?
}
export -f check_formatting

# Loop through all c/cpp/proto files and check validation
find . -regex '.*\.\(c\|cc\|cpp\|cxx\|hpp\|ipp\|proto\)$' \
     | parallel --joblog formatting.log -j$(nproc) check_formatting
# Count how many returned a non zero exist status
ret=$(tail -n +2 formatting.log | awk '{ sum += $7; } END {print sum}')

echo "$ret files are not formatted correctly"
exit $ret
