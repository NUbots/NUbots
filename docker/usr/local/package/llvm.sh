#!/bin/bash

# Exit immediately on error
set -e

# Set installation prefix, default to /usr/local unless an external power says otherwise
PREFIX=${PREFIX:-"/usr/local"}

BUILD_FOLDER="/var/tmp/build"

# Pull in the toolchain arguments
. "${PREFIX}/toolchain.sh"

# Set up the temporary build directories
mkdir -p "${BUILD_FOLDER}"
cd "${BUILD_FOLDER}"

git clone https://github.com/intel/vc-intrinsics vc-intrinsics

# download-and-extract https://github.com/llvm/llvm-project/archive/llvmorg-10.0.0.tar.gz && mv llvm-project-llvmorg-10.0.0 llvm-project

git clone -b release/10.x https://github.com/llvm/llvm-project llvm-project
cd llvm-project
git checkout -b tag llvmorg-10.0.0
cd ..

download-and-extract https://github.com/intel/opencl-clang/archive/v10.0.0-2.tar.gz && mv opencl-clang-10.0.0-2 llvm-project/llvm/projects/opencl-clang

# download-and-extract https://github.com/KhronosGroup/SPIRV-LLVM-Translator/archive/v10.0.0.tar.gz && mv SPIRV-LLVM-Translator-10.0.0 llvm-project/llvm/projects/llvm-spirv

git clone -b llvm_release_100 https://github.com/KhronosGroup/SPIRV-LLVM-Translator llvm-project/llvm/projects/llvm-spirv

download-and-extract https://github.com/intel/intel-graphics-compiler/archive/igc-1.0.5964.tar.gz && mv intel-graphics-compiler-igc-1.0.5964 igc
git clone https://github.com/intel/llvm-patches llvm_patches

mkdir build
cd build
cmake ../igc/IGC
#     -DCMAKE_BUILD_TYPE="Release" \
#     -DCMAKE_TOOLCHAIN_FILE="/usr/local/toolchain.cmake" \
#     -Wno-dev

make -j$(nproc)

make install
