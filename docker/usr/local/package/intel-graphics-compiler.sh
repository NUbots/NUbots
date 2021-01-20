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

echo "Configuring using cmake"

# Find the closest configure file to the root
CMAKELISTS_FILE=$(find -type f -name 'CMakeLists.txt' -printf '%d\t%P\n' | sort -nk1 | cut -f2- | head -n 1)
cd $(dirname "${CMAKELISTS_FILE}")

echo "Configuring using cmake file ${CMAKELISTS_FILE}"

# Clone the dependency
git clone https://github.com/intel/vc-intrinsics vc-intrinsics

# Do an out of source build
mkdir -p build
cd build

# Configure using cmake
cmake .. "$@" -GNinja \
    -DCMAKE_BUILD_TYPE="Release" \
    -DCMAKE_TOOLCHAIN_FILE="${PREFIX}/toolchain.cmake" \
    -Wno-dev

# Build the package
ninja -j $(nproc)

# Now install
ninja install

# Now that we have built, cleanup the build directory
rm -rf "${BUILD_FOLDER}"
