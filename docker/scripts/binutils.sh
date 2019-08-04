# Make a temporary directory
mkdir /build/temp
cd /build/temp

# Get source code
wget https://ftpmirror.gnu.org/gnu/binutils/binutils-${1}.tar.xz

# Extract source code
tar xf binutils-${1}.tar.xz

# Build library

# Install library/strip/whatever else you want to do

# Delete the temporary directory to clean up
rm -rf /build/temp
