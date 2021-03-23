#!/bin/bash

# Exit immediately on error
set -e

# Get our method as the name of this script and the url
METHOD=$(basename "$0")
URL="$1"
shift
CONFIG_PATH="$1"
shift

# Set installation prefix, default to /usr/local unless an external power says otherwise
PREFIX=${PREFIX:-"/usr/local"}

BUILD_FOLDER="/var/tmp/build"

# Pull in the toolchain arguments
. "${PREFIX}/toolchain.sh"

# Setup the temporary build directories
mkdir -p "${BUILD_FOLDER}"
cd "${BUILD_FOLDER}"

# Download the source code and extract it
download-and-extract "${URL}"

echo "Configuring using cmake"

# Find the closest configure file to the root
CMAKELISTS_FILE=$(find -type f -path "*${BUILD_FILE_DIR}/CMakeLists.txt" -printf '%d\t%P\n' | sort -nk1 | cut -f2- | head -n 1)
cd $(dirname "${CMAKELISTS_FILE}")

echo "Configuring using cmake file ${CMAKELISTS_FILE}"

# Do an out of source build
mkdir -p build
cd build

# Configure using cmake
cmake .. -GNinja \
    "$@" \
    -DCMAKE_BUILD_TYPE="Release" \
    -DCMAKE_TOOLCHAIN_FILE="${PREFIX}/toolchain.cmake" \
    -Wno-dev

# Copy our config file over the generated on
# The generated config file doesn't enable LAPACK because its "too hard" to determine if
# OpenBLAS has included LAPACK or not
cp "${CONFIG_PATH}" "tmp/include/armadillo_bits/config.hpp"

# Run ninja
ninja

# Run ninja install
ninja install

# Make sure our config file makes it into the install location
cp "${CONFIG_PATH}" "${PREFIX}/include/armadillo_bits/config.hpp"

# Now that we have built, cleanup the build directory
rm -rf "${BUILD_FOLDER}"
