#!/usr/bin/bash

extract_tars(){
    cd /l4t

    echo "Extracting target filesystem..."
    cat targetfs.tbz2.* | tar -I lbzip2 -x

    echo "Extracting toolchain..."
    mkdir toolchain
    tar -C toolchain -xf toolchain.tar.bz2

    echo "Cleaning up tarballs..."
    rm targetfs.tbz2.* toolchain.tar.bz2
}

strip_sysroot(){
    echo "Stripping sysroot..."

    # Desktop applications
    rm -rf \
        /l4t/targetfs/usr/lib/libreoffice \
        /l4t/targetfs/usr/lib/thunderbird \
        /l4t/targetfs/usr/lib/snapd \
        /l4t/targetfs/usr/lib/mono \
        /l4t/targetfs/usr/lib/xorg \
        /l4t/targetfs/usr/lib/cups \
        /l4t/targetfs/usr/lib/git-core

    # Kernel/firmware blobs
    rm -rf \
        /l4t/targetfs/usr/lib/firmware \
        /l4t/targetfs/usr/lib/modules

    # Debug symbols
    rm -rf \
        /l4t/targetfs/usr/lib/debug

    # Graphics/browser/scripting libs
    rm -rf \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/dri \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/libwebkit2gtk* \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/libjavascriptcoregtk* \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/guile \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/perl \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/libgs* \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/samba \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/libLLVM* \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/openblas-pthread

    # TensorRT - not used
    rm -rf \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/libnvinfer* \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/libnvonnxparser* \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/libnvparsers*

    # cuDNN - not used
    rm -rf \
        /l4t/targetfs/usr/lib/aarch64-linux-gnu/libcudnn*

    # apt cache from sysroot assembly
    rm -rf \
        /l4t/targetfs/var/cache \
        /l4t/targetfs/var/lib/apt

    echo "Done. Sysroot stripped."
}

setup_env(){
    export DEBIAN_FRONTEND=noninteractive
}

extract_tars
strip_sysroot
setup_env
