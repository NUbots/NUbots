#!/bin/sh

ret=0

# Loop through all c/cpp/proto files and check validation
for src in $(find . -regex '.*\.\(c\|cc\|cpp\|cxx\|hpp\|ipp\|proto\)')
do
    echo "Validating formatting for ${src}"
    clang-format -style=file "${src}" | colordiff -u "${src}" -
    ret=$((ret + $?))
done

echo "$ret files are not formatted correctly"
if [ $ret -eq 0 ]
then
    exit 0
else
    exit 1
fi
