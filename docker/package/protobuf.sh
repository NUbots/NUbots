#!/bin/sh
set -e

# Standard autotools install
VERSION=3.9.1
install-autotools.sh \
    https://github.com/google/protobuf/releases/download/v${VERSION}/protobuf-cpp-${VERSION}.tar.gz \
    --with-zlib
