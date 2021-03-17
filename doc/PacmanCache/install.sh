#!/usr/bin/env bash

# Download docker-compose.yml for package cache (using nroi/flexo)
wget https://raw.githubusercontent.com/NUbots/NUbots/deKoeyer/pacman-cache/doc/PacmanCache/docker-compose.yml

# Start container
docker-compose up -d

# Show last created container
docker container ls -l
