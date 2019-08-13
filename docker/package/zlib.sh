#!/bin/sh
set -e

# Standard autotools install
VERSION=1.2.11
install-autotools.sh \
    https://www.zlib.net/zlib-${VERSION}.tar.gz
