#!/bin/bash

check_formatting() {
    echo "Validating formatting for $1"
    cmake-format "$1" | colordiff --color=yes -u "$1" -
    return $?
}
export -f check_formatting

# Loop through all cmake and role files and check validation
git ls-files | grep '.*\(CMakeLists\.txt\|cmake\|role\)$' \
    | parallel --joblog formatting.log -j$(nproc) check_formatting
# Count how many returned a non zero exist status
ret=$(tail -n +2 formatting.log | awk '{ sum += $7; } END {print sum}')

echo "$ret files are not formatted correctly"
exit $ret
