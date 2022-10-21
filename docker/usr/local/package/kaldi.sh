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
. "${PREFIX}/toolchain.sh"

# Setup the temporary build directories
mkdir -p "${BUILD_FOLDER}"
cd "${BUILD_FOLDER}"

# Download the source code
download-and-extract "${URL}"

# Find the closest configure file to the root
CMAKELISTS_FILE=$(find -type f -name 'CMakeLists.txt' -printf '%d\t%P\n' | sort -nk1 | cut -f2- | head -n 1)
cd $(dirname "${CMAKELISTS_FILE}")

# Patch sources
echo "Patching source"
wget "https://gist.githubusercontent.com/Bidski/e78d3bfee0a2c29760777ddb35e6b75a/raw/dfb4a3213b759cb66f3c3b15623fcf8861cf5cc3/kaldi.patch" -O - | patch -Np1

wget "https://gist.githubusercontent.com/Bidski/b9f5b717d20506c634368761daf0db4b/raw/1488eb7a31089cc34093419b0c38b83b12c97ed8/FindOpenFST.cmake" -O cmake/FindOpenFST.cmake

echo "Configuring using cmake file ${CMAKELISTS_FILE}"

# Do an out of source build
mkdir -p build
cd build

# Configure using cmake
cmake .. "$@" \
    -DCMAKE_BUILD_TYPE="Release" \
    -DCMAKE_TOOLCHAIN_FILE="${PREFIX}/toolchain.cmake" \
    -Wno-dev \
    -GNinja

# Run ninja
ninja

# Now install
ninja install

# Now that we have built, cleanup the build directory
rm -rf "${BUILD_FOLDER}"
