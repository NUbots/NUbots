#!/bin/sh

check_formatting() {
    echo "Validating formatting for $1"
    cat "$1" | black -q - | colordiff --color=yes -u "$1" -
    return $?
}
export -f check_formatting

# Loop through all python files and check validation
find . -regex '.*\.py$' \
     | parallel --joblog formatting.log -j$(nproc) check_formatting
# Count how many returned a non zero exist status
ret=$(tail -n +2 formatting.log | awk '{ sum += $7; } END {print sum}')

echo "$ret files are not formatted correctly"
exit $ret
