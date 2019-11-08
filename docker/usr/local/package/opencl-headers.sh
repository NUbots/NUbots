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

echo "Installing header files"

# Find the folder containing the header files
CL_FOLDER=$(find -type d -name "CL")
cd "${CL_FOLDER}"

# Remove unneeded header files
rm {cl_d3d,cl_dx9}*.h

# Now install
install -dm755 "${PREFIX}/include/CL"
for header in *.h
do
  install -m 644 "${header}" "${PREFIX}/include/CL/"
done

# Now that we have built, cleanup the build directory
rm -rf "${BUILD_FOLDER}"
