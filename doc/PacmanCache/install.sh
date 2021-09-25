#!/usr/bin/env bash

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
