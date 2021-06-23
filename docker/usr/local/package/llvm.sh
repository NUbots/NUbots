#!/bin/bash

# Exit immediately on error
set -e

# Get our method as the name of this script, url and args
URL="$1"
shift

if [ "${PLATFORM}" == "g4dnxlarge" ]; then
    # Build llvm with cross compile args for aws
    sh /usr/local/bin/install-cmake-from-source "${URL}" \
        -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra" \
        -DLLVM_ENABLE_SPHINX=OFF \
        -DLLVM_BUILD_LLVM_DYLIB=ON \
        -DLLVM_LINK_LLVM_DYLIB=ON \
        -DCMAKE_CROSSCOMPILING=True \
        # -DCMAKE_INSTALL_PREFIX=<install-dir> \ ??
        -DLLVM_TABLEGEN=/usr/local/bin/llvm-tblgen \
        -DCLANG_TABLEGEN=/usr/local/bin/clang-tblgen \
        -DLLVM_DEFAULT_TARGET_TRIPLE=x86_64-pc-linux-gnu \
        -DLLVM_TARGET_ARCH=x86_64 \
        -DLLVM_TARGETS_TO_BUILD=x86_64 \
else
    # Build llvm normally :)
    sh /usr/local/bin/install-cmake-from-source "${URL}" \
        -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra" \
        -DLLVM_ENABLE_SPHINX=OFF \
        -DLLVM_BUILD_LLVM_DYLIB=ON \
        -DLLVM_LINK_LLVM_DYLIB=ON
fi
