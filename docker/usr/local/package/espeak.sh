#!/bin/bash

# Exit immediately on error
set -e

# Get our method as the name of this script and the url
METHOD=$(basename "$0")
URL="$1"
shift

# Set installation prefix, default to /usr/local unless an external power says otherwise
PREFIX=${PREFIX:-"/usr/local"}

BUILD_FOLDER="/var/tmp/build"

# Dont pull in the toolchain arguments as they seem to break espeaks build
# . "${PREFIX}/toolchain.sh"

# Setup the temporary build directories
mkdir -p "${BUILD_FOLDER}"
cd "${BUILD_FOLDER}"

# Download the source code
download-and-extract "${URL}"

echo "Configuring using autotools"

# Find the closest configure file to the root
AUTOGEN_FILE=$(find -type f -path "*${BUILD_FILE_DIR}/autogen.sh" -printf '%d\t%P\n' | sort -nk1 | cut -f2- | head -n 1)
cd $(dirname "${AUTOGEN_FILE}")

echo "Configuring using configure file ${AUTOGEN_FILE}"

# We have to run ldconfig as it wants to do things with the libraries
sudo ldconfig

# Run autogen
./autogen.sh

# Run configure, No CFlags in here as it seems that espeak hates them so much
./configure "$@" --prefix="${PREFIX}"

# Whoever wrote the build system for espeak is really bad at this
# Unless you step make through the process carefully race conditions cause unrecoverable errors
make src/espeak-ng -j1
make src/speak-ng -j1
make -j1

# Run make install
make install

# Now that we have built, cleanup the build directory
rm -rf "${BUILD_FOLDER}"
