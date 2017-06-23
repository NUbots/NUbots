# Define environment variables
PREFIX=/nubots/toolchain

export LD_LIBRARY_PATH="$PREFIX/lib"
export PATH="$PREFIX/bin:$PATH"
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig"
export CMAKE_PREFIX_PATH="$PREFIX"
