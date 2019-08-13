#!/bin/sh
set -e

# Pull out the URL and the configure arguments
URL="$1"
shift
ARGS="$@"

# Pull in the toolchain arguments
. /usr/local/toolchain.sh

# Setup build directories
mkdir -p /var/tmp/build
cd /var/tmp/build

# Download and extract
wget ${URL}
ARCHIVE_FILE=$(find . -type f | head -n 1)
tar xf ${ARCHIVE_FILE}
CONFIG_FILE=$(find . -type f -name "configure" | head -n 1)
CONFIG_DIR=$(dirname ${CONFIG_FILE})
cd ${CONFIG_DIR}

# Configure
./configure --prefix=/usr/local ${ARGS}

# Build and install
make -j$(nproc)
make install

# Cleanup
rm -rf /var/tmp/build
