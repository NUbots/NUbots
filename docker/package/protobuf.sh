#!/bin/sh
set -e

VERSION=3.9.1

# Load toolchain settings
. /usr/local/toolchain.sh

# Setup build directories
mkdir -p /var/tmp/build
cd /var/tmp/build

# Download and extract
wget https://github.com/google/protobuf/releases/download/v${VERSION}/protobuf-cpp-${VERSION}.tar.gz
tar xf protobuf-cpp-${VERSION}.tar.gz
cd protobuf-${VERSION}

./configure --with-zlib --prefix=/usr/local
make -j$(nproc)
make install

# Cleanup
rm -rf /var/tmp/build
