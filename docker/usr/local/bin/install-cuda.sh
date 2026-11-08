#!/usr/bin/bash
##
## MIT License
##
## Copyright (c) 2026 NUbots
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

    apt-get update
fi

rm -rf /var/lib/apt/lists/*
