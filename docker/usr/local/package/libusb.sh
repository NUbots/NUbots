#!/bin/bash

# Exit immediately on error
set -e

# Get our method as the name of this script, url and args
URL="$1"
shift

# Set installation prefix, default to /usr/local unless an external power says otherwise
PREFIX=${PREFIX:-"/usr/local"}

BUILD_FOLDER="/var/tmp/build"

# Pull in the toolchain arguments
. /usr/local/toolchain.sh

# Setup the temporary build directories
mkdir -p "${BUILD_FOLDER}"
cd "${BUILD_FOLDER}"

# Download the source code
download-and-extract "${URL}"

BOOTSTRAPFILE=$(find -type f -path "*${BUILD_FILE_DIR}/bootstrap.sh" -printf '%d\t%P\n' | sort -nk1 | cut -f2- | head -n 1)
cd $(dirname "${BOOTSTRAPFILE}")

# Creates the correct configure file by calling autoreconf
./bootstrap.sh

echo "Configuring using flags from Dockerfile"

./configure "$@"

echo "Building using make"

make -j$(nproc) \
    CFLAGS="${CFLAGS}" \
    CXXFLAGS="${CXXFLAGS}" \
    FFLAGS="${FFLAGS}" \
    FCFLAGS="${FCFLAGS}"

make PREFIX="${PREFIX}" install

# Now that we have built, cleanup the build directory
rm -rf "${BUILD_FOLDER}"
