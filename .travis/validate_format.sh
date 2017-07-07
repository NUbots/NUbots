#!/bin/bash

find . -type f \( -name *.h -o -name *.c -o -name *.cc -o -name *.cxx -o -name *.cpp -o -name *.hpp -o -name *.ipp \) -print0 | while IFS= read -r -d $'\0' line; do
    echo $line
done
