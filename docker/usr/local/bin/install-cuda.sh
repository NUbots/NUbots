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
    # sbsa repo: native arm64 packages (for extracting into the target sysroot)
    wget -nv https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/sbsa/cuda-keyring_1.1-1_all.deb \
        -O cuda-keyring-arm.deb
    dpkg -i cuda-keyring-arm.deb
    rm cuda-keyring-arm.deb

    # cross-linux-sbsa repo: aarch64 CUDA libraries installed on the HOST for cross-compilation
    # (provides /usr/local/cuda-12.5/targets/sbsa-linux/ that nvcc needs when targeting aarch64)
    wget -nv https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/cross-linux-sbsa/cuda-keyring_1.1-1_all.deb \
        -O cuda-keyring-cross.deb
    dpkg -i cuda-keyring-cross.deb
    rm cuda-keyring-cross.deb

    apt-get update
    apt-get install -y --no-install-recommends cuda-cross-sbsa-12-5
fi

rm -rf /var/lib/apt/lists/*
