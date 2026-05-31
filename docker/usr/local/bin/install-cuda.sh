#!/usr/bin/bash

set -e

PLATFORM=$1

# Add x86_64 CUDA keyring (needed for both native build tools and cross-compilation)
wget -nv https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
    -O cuda-keyring.deb
dpkg -i cuda-keyring.deb
rm cuda-keyring.deb

apt-get update

if [ "${PLATFORM}" != "generic" ]; then
    wget -nv https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/sbsa/cuda-keyring_1.1-1_all.deb \
        -O cuda-keyring-arm.deb
    dpkg -i cuda-keyring-arm.deb
    rm cuda-keyring-arm.deb
    apt-get update
fi

rm -rf /var/lib/apt/lists/*
