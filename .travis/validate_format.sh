#!/bin/bash

find . -type f -iname *.h -o -iname *.c -o -iname *.cc -o -iname *.cxx -o -iname *.cpp -o -iname *.hpp -o -iname *.ipp -print0 | while IFS= read -r -d $'\0' line; do
    echo $line
done
