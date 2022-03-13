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
# Lock to specific version so future upstream changes do not interfer with our builds.
git -C vc-intrinsics/ checkout a2f2f10dc61c8161c57cf33ed606c8e3ccf3a921

# TODO(DevOpsTeam): Upgrade IGC version once they support clang12+ properly and remove these patches
# Download and install the patches. The list of patches is from this LEGEND:
# https://github.com/intel/intel-graphics-compiler/issues/191#issuecomment-899753344
# llvm12-porting.patch from arch PKGBUILD for IGC
wget https://raw.githubusercontent.com/archlinux/svntogit-community/fc78567191acd6d23c2a46d8f5de331a820584b4/trunk/llvm12-porting.patch -O - | patch -p1
# Revert stack-analysis
wget https://github.com/intel/intel-graphics-compiler/commit/c50d77f3c8a92fae6f43b47580e7d485f9ea5327.patch -O - | patch -p1 -R
# igc-missing-limits-headers - one line patch for missing include
wget https://github.com/intel/intel-graphics-compiler/commit/8e1a461d3e6b85a6cf018caf6abf4a3ba9a1758d.patch -O - | patch -p1
# fix-dce.patch - Patch by that guy to comment out a troublesome function call
wget https://github.com/intel/intel-graphics-compiler/commit/fd480731ad77213fbfcd9677d03e8243acbb306c.patch -O - | patch -p1

# The build requires a binary in here to be run (CMCLTranslatorTool)
# Without this directory in the path, bash can't find it
PATH="${PATH}:$(pwd)/build/IGC/Release"

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
