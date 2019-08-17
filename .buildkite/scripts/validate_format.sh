#!/bin/sh

check_clang_format() {
    echo "Validating formatting for $1"
    clang-format -style=file "$1" | colordiff --color=yes -u "$1" -
    return $?
}
check_cmake_format() {
    echo "Validating formatting for $1"
    cmake-format "$1" | colordiff --color=yes -u "$1" -
    return $?
}
check_python_format() {
    echo "Validating formatting for $1"
    cat "$1" | black -q - | colordiff --color=yes -u "$1" -
    return $?
}
export -f check_clang_format
export -f check_cmake_format
export -f check_python_format

# Loop through all c/cpp/proto files and check validation
find . -regex '.*\.\(c\|cc\|cpp\|cxx\|hpp\|ipp\|proto\)$' \
     | parallel --joblog formatting.log -j$(nproc) check_clang_format
# Count how many returned a non zero exist status
clang_ret=$(tail -n +2 formatting.log | awk '{ sum += $7; } END {print sum}')

find . -name 'CMakeLists.txt' -o -regex '.*\.\(cmake\|role\)$' \
     | parallel --joblog formatting.log -j$(nproc) check_cmake_format
# Count how many returned a non zero exist status
cmake_ret=$(tail -n +2 formatting.log | awk '{ sum += $7; } END {print sum}')

find . -regex '.*\.py$' \
     | parallel --joblog formatting.log -j$(nproc) check_python_format
# Count how many returned a non zero exist status
python_ret=$(tail -n +2 formatting.log | awk '{ sum += $7; } END {print sum}')

ret=$((${clang_ret} + ${cmake_ret} + ${python_ret}))

echo "$ret files are not formatted correctly"
if [ $ret -eq 0 ]
then
    exit 0
else
    exit 1
fi
