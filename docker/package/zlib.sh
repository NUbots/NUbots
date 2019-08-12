#!/bin/sh
set -e

VERSION=1.2.11

# Load toolchain settings
. /usr/local/toolchain.sh

# Setup build directories
mkdir -p /var/tmp/build
cd /var/tmp/build

# Download and extract
wget https://www.zlib.net/zlib-${VERSION}.tar.gz
tar xf zlib-${VERSION}.tar.gz
cd zlib-${VERSION}

./configure --prefix=/usr/local
make -j$(nproc)
make install

# Cleanup
rm -rf /var/tmp/build
