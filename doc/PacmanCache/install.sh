#!/usr/bin/env bash
##
## MIT License
##
## Copyright (c) 2021 NUbots
##
## This file is part of the NUbots codebase.
## See https://github.com/NUbots/NUbots for further info.
##
## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in all
## copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
## AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
## OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
## SOFTWARE.
##

URL=https://raw.githubusercontent.com/NUbots/NUbots/main/doc/PacmanCache/docker-compose.yml
FILE_NAME=docker-compose.yml

# Download docker-compose.yml for package cache (using nroi/flexo)
if [ -x "$(which wget)" ] ; then
    wget $URL -O $FILE_NAME

elif [ -x "$(which curl)" ]; then
    curl -o $FILE_NAME $URL

else
    echo "Neither wget or curl installed!"
    exit 1

fi

# Start container
docker-compose up -d

# Show last created container
docker container ls -l
