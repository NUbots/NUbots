#!/bin/sh

# Set our compilers
export CC=/usr/bin/gcc
export CXX=/usr/bin/g++
export FC=/usr/bin/gfortran

# Set our package config so it finds things in the toolchain
export PKG_CONFIG_PATH="/usr/local/lib/pkgconfig"

# Set our optimisation flags
export CFLAGS="-mtune=generic"
export CXXFLAGS=${CFLAGS}
export CPPFLAGS=${CFLAGS}
export FFLAGS=${CFLAGS}
