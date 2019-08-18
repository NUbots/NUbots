##############################################
#   ___  ____    ____       _                #
#  / _ \/ ___|  / ___|  ___| |_ _   _ _ __   #
# | | | \___ \  \___ \ / _ \ __| | | | '_ \  #
# | |_| |___) |  ___) |  __/ |_| |_| | |_) | #
#  \___/|____/  |____/ \___|\__|\__,_| .__/  #
#                                    |_|     #
##############################################
# FROM alpine:edge
# RUN apk update && apk add --no-cache build-base libstdc++
# RUN addgroup -S nubots && adduser -S nubots -G nubots

# FROM ubuntu:18.04
# RUN apt-get update && apt-get -y install build-essential autotools-dev autoconf automake g++ gdb valgrind gfortran wget yasm
# RUN groupadd -r nubots && useradd --no-log-init -r -g nubots nubots

FROM archlinux/base:latest
RUN pacman -Syu --noconfirm --needed \
    && pacman -S --noconfirm --needed \
    wget \
    python \
    python-pip \
    base-devel \
    gcc-fortran \
    yasm \
    gdb \
    valgrind \
    ninja \
    cmake \
    meson \
    git \
    && rm -rf /var/cache \
    && sed "s/^\(PREFIXES\s=\s\)\[\([^]]*\)\]/\1[\2, '\/usr\/local']/" -i /usr/lib/python3.7/site.py
RUN groupadd -r nubots && useradd --no-log-init -r -g nubots nubots

# Create the home directory owned by nubots
RUN mkdir -p /home/nubots && chown -R nubots:nubots /home/nubots

# Setup /usr/local owned by nubots and swap to the nubots user
RUN chown -R nubots:nubots /usr/local
USER nubots

# Add ssh key for sshing into the robot
COPY --chown="nubots:nubots" "files/id_rsa" "/home/nubots/.ssh/id_rsa"
COPY --chown="nubots:nubots" "files/id_rsa.pub" "/home/nubots/.ssh/id_rsa.pub"
COPY --chown="nubots:nubots" "files/ssh_config" "/home/nubots/.ssh/ssh_config"

# Copy across the generic toolchain file for building tools
COPY --chown=nubots:nubots toolchain/generic.sh /usr/local/toolchain.sh

# Copy over a tool to install simple standard conforming libraries from source
COPY --chown=nubots:nubots package/install-from-source /usr/local/bin/install-from-source
RUN ln -s /usr/local/bin/install-from-source /usr/local/bin/install-header-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-cmake-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-autotools-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-bjam-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-make-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-meson-from-source

# Install build tools
# RUN install-from-source https://github.com/Kitware/CMake/releases/download/v3.15.2/cmake-3.15.2.tar.gz
# https://github.com/ninja-build/ninja/archive/v1.9.0.tar.gz

################################################
#  _____           _      _           _        #
# |_   _|__   ___ | | ___| |__   __ _(_)_ __   #
#   | |/ _ \ / _ \| |/ __| '_ \ / _` | | '_ \  #
#   | | (_) | (_) | | (__| | | | (_| | | | | | #
#   |_|\___/ \___/|_|\___|_| |_|\__,_|_|_| |_| #
################################################
ARG platform=generic

# Copy across the specific toolchain file for this image
COPY --chown=nubots:nubots toolchain/${platform}.sh /usr/local/toolchain.sh

# zlib
RUN install-from-source https://www.zlib.net/zlib-1.2.11.tar.gz

# OpenBLAS
RUN if [ "${platform}" = "generic" ] ; \
    then \
    install-make-from-source https://github.com/xianyi/OpenBLAS/archive/v0.3.7.tar.gz \
    BINARY=64 \
    SMP=1 \
    NUM_THREADS=2 \
    DYNAMIC_ARCH=1 \
    TARGET=GENERIC \
    USE_THREAD=1  \
    NO_SHARED=0 \
    NO_STATIC=0 ; \
    else \
    install-make-from-source https://github.com/xianyi/OpenBLAS/archive/v0.3.7.tar.gz \
    CROSS=1 \
    BINARY=64 \
    SMP=1 \
    NUM_THREADS=2 \
    TARGET=HASWELL \
    USE_THREAD=1 \
    NO_SHARED=0 \
    NO_STATIC=0 ; \
    fi

# Armadillo
RUN install-cmake-from-source http://sourceforge.net/projects/arma/files/armadillo-9.600.6.tar.xz \
    -DDETECT_HDF5=OFF \
    -DBUILD_SHARED_LIBS=ON
COPY --chown=nubots:nubots package/armadillo_config.hpp /usr/local/include/armadillo_bits/config.hpp

# Eigen3
RUN install-from-source http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2

# tcmalloc
RUN install-from-source https://github.com/gperftools/gperftools/releases/download/gperftools-2.7/gperftools-2.7.tar.gz \
    --with-tcmalloc-pagesize=64 \
    --enable-minimal

# Protobuf
RUN install-from-source https://github.com/google/protobuf/releases/download/v3.9.1/protobuf-cpp-3.9.1.tar.gz \
    --with-zlib

# Libjpeg
RUN install-from-source https://github.com/libjpeg-turbo/libjpeg-turbo/archive/2.0.2.tar.gz

# yaml-cpp
RUN install-from-source https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.2.tar.gz \
    -DYAML_CPP_BUILD_TESTS=OFF \
    -DBUILD_SHARED_LIBS=OFF

# fmt formatting library
RUN  install-from-source https://github.com/fmtlib/fmt/archive/5.3.0.tar.gz \
    -DFMT_DOC=OFF \
    -DFMT_TEST=OFF

# Catch unit testing library
RUN install-header-from-source https://github.com/catchorg/Catch2/releases/download/v2.9.2/catch.hpp

# Aravis
RUN install-from-source http://xmlsoft.org/sources/libxml2-2.9.3.tar.gz --with-zlib=/usr/local --without-python
RUN install-from-source https://github.com/libffi/libffi/archive/v3.3-rc0.tar.gz
RUN install-from-source https://mirrors.edge.kernel.org/pub/linux/utils/util-linux/v2.34/util-linux-2.34.tar.xz \
    --disable-all-programs \
    --enable-libblkid \
    --enable-libmount \
    --enable-libuuid
COPY --chown=nubots:nubots package/${platform}_glib.cross /var/tmp/glib.cross
RUN install-from-source https://gitlab.gnome.org/GNOME/glib/-/archive/2.61.2/glib-2.61.2.tar.gz \
    --cross-file=/var/tmp/glib.cross \
    -Ddefault_library=both \
    -Dinternal_pcre=true \
    && cp /usr/local/lib/glib-2.0/include/glibconfig.h /usr/local/include/glibconfig.h
RUN install-meson-from-source https://github.com/AravisProject/aravis/archive/ARAVIS_0_6_3.tar.gz \
    -Ddefault_library=both \
    -Dviewer=false \
    -Dgst-plugin=false \
    -Dusb=true \
    -Ddocumentation=false \
    -Dintrospection=false

# FSWatch
RUN install-from-source https://github.com/emcrisostomo/fswatch/releases/download/1.14.0/fswatch-1.14.0.tar.gz

# LibUV
RUN install-cmake-from-source https://github.com/libuv/libuv/archive/v1.31.0.tar.gz \
    -Dlibuv_buildtests=OFF \
    -DBUILD_TESTING=OFF

# NUClear!
RUN install-from-source https://github.com/Fastcode/NUClear/archive/master.tar.gz \
    -DBUILD_TESTS=OFF

# LibBacktrace
RUN install-from-source https://github.com/ianlancetaylor/libbacktrace/archive/master.tar.gz \
    --without-system-libunwind \
    --enable-shared \
    --enable-static

# Setup pip to install to /usr/local and have python find packages there
ENV PYTHONPATH=/usr/local/lib/python3.7/site-packages
COPY --chown=root:root files/pip.conf /etc/pip.conf

# Install python libraries
RUN pip install \
    protobuf==3.9.1 \
    stringcase

# http://registrationcenter-download.intel.com/akdlm/irc_nas/11396/SRB4.1_linux64.zip
# or https://01.org/compute-runtime

# http://www.fftw.org/fftw-3.3.6-pl2.tar.gz
# http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz
# https://dl.bintray.com/boostorg/release/1.64.0/source/boost_1_64_0.tar.gz

# Go to where we will mount the NUbots volume
WORKDIR /home/nubots/NUbots
