##
## MIT License
##
## Copyright (c) 2019 NUbots
##
## This file is part of the NUbots codebase.
## See https://github.com/NUbots/NUbots for further info.
##
## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in all
## copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
## AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
## OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
## SOFTWARE.
##

# syntax=docker/dockerfile:1.3
# For information on "RUN --mount ..." syntax
# See https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/syntax.md

##############################################
#   ___  ____    ____       _                #
#  / _ \/ ___|  / ___|  ___| |_ _   _ _ __   #
# | | | \___ \  \___ \ / _ \ __| | | | '_ \  #
# | |_| |___) |  ___) |  __/ |_| |_| | |_) | #
#  \___/|____/  |____/ \___|\__|\__,_| .__/  #
#                                    |_|     #
##############################################
### The following two commands are paired
### The date in the Server command must either match, or be more recent than,
### the date in the FROM command
### Date format is ISO year/month/day
FROM --platform=linux/amd64 archlinux:base-devel-20241117.0.280007

COPY etc/pacman.d/mirrorlist /etc/pacman.d/mirrorlist

# Setup pacman first before we install so we can turn off pgp
COPY etc/pacman.conf /etc/pacman.conf

# Arch recommends running this as the first command of the docker image
# Use "yyuu" to allow for downgrades
# There wont be any downgrades provided the date in Server line above is the same
# or newer than the arch image date
RUN pacman -Syyuu --noconfirm --needed --overwrite \*

# Add a script that installs packages
COPY usr/local/bin/install-package /usr/local/bin/install-package

# Install base packages needed for building general toolchain
# If you have a tool that's needed for a specific module install it before that module
RUN install-package \
    wget \
    sudo \
    python \
    python-pip \
    python-tqdm \
    python-requests \
    base-devel \
    ccache \
    ninja \
    cmake \
    meson \
    git \
    zlib \
    openssh \
    rsync \
    gdb \
    colordiff \
    parallel \
    vim \
    nano

# Get python to look in /usr/local for packages
RUN echo $(python -c "import site; print(site.getsitepackages()[0].replace('/usr', '/usr/local'))") \
    > $(python -c "import site; print(site.getsitepackages()[0])")/local.pth
COPY etc/xdg/pip/pip.conf /etc/xdg/pip/pip.conf

# Make sure /usr/local is checked for libraries and binaries
COPY etc/ld.so.conf.d/usrlocal.conf /etc/ld.so.conf.d/usrlocal.conf
RUN ldconfig

# Make a symlink from /usr/local/lib to /usr/local/lib64 so library install location is irrelevant
RUN cd /usr/local && ln -sf lib lib64

# Copy over a tool to install simple standard conforming libraries from source
COPY usr/local/bin/install-from-source.py /usr/local/bin/install-from-source.py
RUN ln -s /usr/local/bin/install-from-source.py /usr/local/bin/install-from-source

# Generate toolchain files for generic
RUN --mount=type=bind,source=usr/local/toolchain,target=/usr/local/toolchain \
    python /usr/local/toolchain/generate_generic_toolchain.py --prefix /usr

#############################################
### ADD NEW SYTEM PROGRAMS/LIBRARIES HERE ###
#############################################

RUN pip install python-magic==0.4.27

################################################
#  _____           _      _           _        #
# |_   _|__   ___ | | ___| |__   __ _(_)_ __   #
#   | |/ _ \ / _ \| |/ __| '_ \ / _` | | '_ \  #
#   | | (_) | (_) | | (__| | | | (_| | | | | | #
#   |_|\___/ \___/|_|\___|_| |_|\__,_|_|_| |_| #
################################################
ARG platform=generic
ARG prefix=/usr/local

# Generate toolchain files for the current platform
RUN --mount=type=bind,source=usr/local/toolchain,target=/usr/local/toolchain \
    python /usr/local/toolchain/generate_${platform}_toolchain.py --prefix ${prefix}

# LLVM and Clang
RUN install-from-source https://github.com/llvm/llvm-project/archive/refs/tags/llvmorg-14.0.5.tar.gz \
    --build-system cmake \
    --configure-path "llvm" \
    -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra;lld" \
    -DLLVM_ENABLE_SPHINX=OFF \
    -DLLVM_BUILD_LLVM_DYLIB=ON \
    -DLLVM_LINK_LLVM_DYLIB=ON

# SPIRV LLVM Translator
RUN install-from-source https://github.com/KhronosGroup/SPIRV-LLVM-Translator/archive/a53b216b970cd101e5019c35d3f3f096459073de.tar.gz \
    --build-system cmake \
    -DBUILD_SHARED_LIBS=ON \
    -DLLVM_INCLUDE_TESTS=OFF

# opencl-clang
RUN install-from-source https://github.com/intel/opencl-clang/archive/470cf0018e1ef6fc92eda1356f5f31f7da452abc.tar.gz \
    --patch https://gist.githubusercontent.com/ysims/2c3f93fd7c0a6edba6594f1564dcfecb/raw/0a00b1c16c23822178170d2b7ae47a1ce0ed343c/resource_linker_skip_cross.patch

# Intel graphics compiler dependencies
RUN install-from-source https://github.com/intel/vc-intrinsics/archive/refs/tags/v0.19.0.tar.gz \
    -DLLVM_DIR=${prefix}/lib/cmake/llvm
RUN install-from-source https://github.com/KhronosGroup/SPIRV-Headers/archive/1c6bb2743599e6eb6f37b2969acc0aef812e32e3.tar.gz
RUN install-from-source https://github.com/KhronosGroup/SPIRV-Tools/archive/refs/tags/v2023.6.rc1.tar.gz \
    -DSPIRV_WERROR=Off \
    -DBUILD_SHARED_LIBS=ON \
    -DSPIRV_TOOLS_BUILD_STATIC=OFF \
    -DSPIRV-Headers_SOURCE_DIR=${prefix}
RUN install-from-source https://download.savannah.nongnu.org/releases/libunwind/libunwind-1.6.2.tar.gz

# We install ncurses rather than building it as it's used by many of the build tools and they don't like it when it changes
RUN install-package ncurses

RUN pip install pyyaml mako

# Intel graphics compiler
RUN install-from-source https://github.com/intel/intel-graphics-compiler/archive/refs/tags/igc-1.0.17791.9.tar.gz \
    --patch https://gist.githubusercontent.com/ysims/3775531601b210507c478bca083b768d/raw/fa435e70c2870f2539c0da973a161ad7968d4696/built_tools_target.patch \
    -DIGC_OPTION__ARCHITECTURE_TARGET='Linux64' \
    -DIGC_OPTION__CLANG_MODE=Prebuilds \
    -DIGC_OPTION__LLD_MODE=Prebuilds \
    -DIGC_OPTION__LLVM_PREFERRED_VERSION='14.0.5' \
    -DIGC_OPTION__LLVM_MODE=Prebuilds \
    -DIGC_OPTION__LINK_KHRONOS_SPIRV_TRANSLATOR=ON \
    -DIGC_OPTION__USE_PREINSTALLED_SPRIV_HEADERS=ON \
    -DIGC_OPTION__SPIRV_TOOLS_MODE=Prebuilds \
    -DIGC_OPTION__SPIRV_TRANSLATOR_MODE=Prebuilds \
    -DIGC_OPTION__VC_INTRINSICS_MODE=Prebuilds \
    -DINSTALL_GENX_IR=ON

# gmmlib
RUN install-from-source https://github.com/intel/gmmlib/archive/refs/tags/intel-gmmlib-22.5.2.tar.gz \
    --build-system cmake \
    -DRUN_TEST_SUITE=OFF

# Libpci access and libdrm for communicating with graphics hardware
RUN install-from-source https://xorg.freedesktop.org/releases/individual/lib/libpciaccess-0.16.tar.bz2
RUN install-from-source https://dri.freedesktop.org/libdrm/libdrm-2.4.112.tar.xz \
    --build-system meson \
    -Dudev=false \
    -Dvalgrind=false

# Libva and the iHD driver for hardware compression
RUN install-from-source https://github.com/intel/libva/archive/refs/tags/2.22.0.tar.gz
RUN install-from-source https://github.com/intel/media-driver/archive/refs/tags/intel-media-24.3.4.tar.gz \
    --build-system cmake \
    -DINSTALL_DRIVER_SYSCONF=OFF \
    -DLIBVA_DRIVERS_PATH="${prefix}/lib/dri" \
    -DMEDIA_RUN_TEST_SUITE=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_KERNELS=ON \
    -DENABLE_KERNELS=ON \
    -DENABLE_NONFREE_KERNELS=ON

# Install intel compute runtime (OpenCL implementation)
RUN install-from-source https://github.com/intel/compute-runtime/archive/refs/tags/24.39.31294.12.tar.gz \
    -DSKIP_UNIT_TESTS=ON \
    -DIGC_DIR=${prefix}

# Install OpenCL C Headers
RUN install-from-source https://github.com/KhronosGroup/OpenCL-Headers/archive/refs/tags/v2022.05.18.tar.gz

# OpenCL loader library
RUN install-package ruby
RUN install-from-source https://github.com/OCL-dev/ocl-icd/archive/refs/tags/v2.3.1.tar.gz

# OpenCV
RUN install-from-source https://github.com/opencv/opencv/archive/refs/tags/4.9.0.tar.gz

# Protobuf
RUN install-package protobuf && \
    install-from-source https://github.com/protocolbuffers/protobuf/releases/download/v21.4/protobuf-cpp-3.21.4.tar.gz \
    --patch https://github.com/protocolbuffers/protobuf/commit/735221ff3a1b8e5ca1a7b38a1884043e25864f31.patch \
    -DWITH_PROTOC="/usr/sbin/protoc" \
    -Dprotobuf_BUILD_TESTS=OFF \
    -Dprotobuf_BUILD_LIBPROTOC=ON \
    -DBUILD_SHARED_LIBS=OFF

# OpenVino - Dependencies
RUN install-package opencl-clhpp \
    shellcheck \
    scons \
    pkgconf \
    patchelf \
    fdupes \
    ca-certificates \
    file \
    rapidjson \
    snappy

# Clone OpenVINO repository
RUN git clone https://github.com/openvinotoolkit/openvino.git --branch 2024.6.0 && \
    cd openvino && \
    git submodule update --init --recursive

# Download AUR patches
WORKDIR /openvino
RUN curl -L -o 010-openvino-change-install-paths.patch \
    https://aur.archlinux.org/cgit/aur.git/plain/010-openvino-change-install-paths.patch?h=openvino && \
    curl -L -o 020-openvino-disable-werror.patch \
    https://aur.archlinux.org/cgit/aur.git/plain/020-openvino-disable-werror.patch?h=openvino && \
    curl -L -o 030-openvino-level-zero-disable-werror.patch \
    https://aur.archlinux.org/cgit/aur.git/plain/030-openvino-level-zero-disable-werror.patch?h=openvino && \
    curl -L -o 040-openvino-protobuf23-fix.patch \
    https://aur.archlinux.org/cgit/aur.git/plain/040-openvino-protobuf23-fix.patch?h=openvino

# Apply patches
RUN cd /openvino && \
    patch -d . -p1 < 010-openvino-change-install-paths.patch && \
    patch -d . -p1 < 020-openvino-disable-werror.patch && \
    patch -d thirdparty/level_zero/level-zero -p1 < 030-openvino-level-zero-disable-werror.patch && \
    patch -d . -p1 < 040-openvino-protobuf23-fix.patch

# Build OpenVINO
RUN mkdir build && cd build && \
    cmake .. -DBUILD_TESTING:BOOL='OFF' \
    -DCMAKE_BUILD_TYPE:STRING='Release' \
    -DENABLE_AVX512F:BOOL='OFF' \
    -DENABLE_PYTHON:BOOL='OFF' \
    -DENABLE_CLANG_FORMAT:BOOL='OFF' \
    -DENABLE_NCC_STYLE:BOOL='OFF' \
    -DENABLE_SYSTEM_PUGIXML:BOOL='ON' \
    -DENABLE_INTEL_NPU:BOOL='OFF' \
    -DENABLE_SYSTEM_TBB:BOOL='ON' \
    -DENABLE_SYSTEM_OPENCL:BOOL='ON' \
    -DENABLE_SYSTEM_PROTOBUF:BOOL='OFF' \
    -DENABLE_SYSTEM_FLATBUFFERS:BOOL='OFF' \
    -DENABLE_CPPLINT:BOOL='OFF' \
    -DCMAKE_CXX_STANDARD='17' \
    -Wno-dev && \
    make -j$(nproc) && make install

# protobuf v21 lots of errors
# protobuf v22 produces error error: ‘uint32_t’ was not declared in this scope
# protobuf v23 produces error error: ‘SetGlobalVLogLevel’ is not a member of ‘absl’
# protobuf v24 produces error error: ‘SetGlobalVLogLevel’ is not a member of ‘absl’
# v25 has CMake error with protobuf "/protobuf/java/core/src/main/java/com/google/java_features.proto": No such file or directory.\
# v26 has OpenVINO error relocation R_X86_64_TPOFF32 against hidden symbol `_ZN6google8protobuf8internal22allocate_at_least_hookE' can not be used when making a shared object
# v27 has fatal error: google/protobuf/stubs/logging.h: No such file or directory
# v28 fatal error: google/protobuf/stubs/logging.h: No such file or directory
# v29 fatal error: google/protobuf/stubs/logging.h: No such file or directory

# Move OpenVino libraries to our library path
# RUN sudo cp /usr/local/runtime/lib/intel64/* /usr/local/lib/ -r && sudo cp /usr/local/runtime/3rdparty/tbb/lib/* /usr/local/lib/ -r

# Eigen3
RUN install-from-source https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz

# tcmalloc
RUN install-from-source https://github.com/gperftools/gperftools/releases/download/gperftools-2.10/gperftools-2.10.tar.gz \
    --build-system autotools \
    --with-tcmalloc-pagesize=64 \
    --enable-minimal

# Libjpeg
RUN install-package yasm
RUN install-from-source https://github.com/libjpeg-turbo/libjpeg-turbo/archive/refs/tags/2.0.7-esr.tar.gz \
    -DWITH_SIMD=ON \
    -DFORCE_INLINE=ON \
    -DINLINE_WORKS=1 \
    -DRIGHT_SHIFT_IS_UNSIGNED_COMPILED=ON \
    -DRIGHT_SHIFT_IS_UNSIGNED_EXITCODE=1 \
    -DRIGHT_SHIFT_IS_UNSIGNED="" \
    -D__CHAR_UNSIGNED___COMPILED=ON \
    -D__CHAR_UNSIGNED___EXITCODE=1 \
    -D__CHAR_UNSIGNED__="" \
    -DFLOATTEST="sse"

# yaml-cpp
RUN install-from-source https://github.com/jbeder/yaml-cpp/archive/refs/tags/yaml-cpp-0.7.0.tar.gz \
    -DYAML_CPP_BUILD_TESTS=OFF \
    -DBUILD_SHARED_LIBS=OFF

# fmt formatting library
RUN install-from-source https://github.com/fmtlib/fmt/archive/refs/tags/9.0.0.tar.gz \
    -DFMT_DOC=OFF \
    -DFMT_TEST=OFF

# Catch unit testing library
RUN install-from-source https://github.com/catchorg/Catch2/archive/refs/tags/v3.3.2.tar.gz \
    -DCATCH_CONFIG_CONSOLE_WIDTH=120 \
    -DCATCH_CONFIG_CPP11_TO_STRING=ON \
    -DCATCH_CONFIG_CPP17_UNCAUGHT_EXCEPTIONS=ON \
    -DCATCH_CONFIG_CPP17_STRING_VIEW=ON \
    -DCATCH_CONFIG_CPP17_VARIANT=ON \
    -DCATCH_CONFIG_CPP17_OPTIONAL=ON \
    -DCATCH_CONFIG_CPP17_BYTE=ON

# Aravis
RUN install-from-source https://github.com/libusb/libusb/archive/refs/tags/v1.0.26.tar.gz
RUN install-from-source http://xmlsoft.org/sources/libxml2-2.9.10.tar.gz \
    --without-python
RUN install-from-source https://github.com/libffi/libffi/releases/download/v3.4.2/libffi-3.4.2.tar.gz
RUN install-package glib2-devel
RUN  ls ${prefix}/include && \
    install-from-source https://github.com/AravisProject/aravis/releases/download/0.8.22/aravis-0.8.22.tar.xz \
    -Ddefault_library=both \
    -Dusb=enabled \
    -Ddocumentation=disabled \
    -Dintrospection=disabled

# LibUV
RUN install-from-source https://github.com/libuv/libuv/archive/refs/tags/v1.44.2.tar.gz \
    -Dlibuv_buildtests=OFF \
    -DBUILD_TESTING=OFF

# NUClear!
RUN install-from-source https://github.com/Fastcode/NUClear/archive/964efc8c1771c67c417d8e683bd09b24d8e2871b.tar.gz \
    -DBUILD_TESTS=OFF

# Backtrace
RUN install-from-source https://github.com/ianlancetaylor/libbacktrace/archive/8602fda64e78f1f46563220f2ee9f7e70819c51d.tar.gz \
    --without-system-libunwind \
    --enable-shared \
    --enable-static

# Alsa and espeak
RUN install-from-source https://github.com/alsa-project/alsa-lib/archive/refs/tags/v1.2.7.2.tar.gz \
    --without-debug
RUN install-package alsa-utils
RUN install-from-source https://github.com/espeak-ng/pcaudiolib/releases/download/1.2/pcaudiolib-1.2.tar.gz
# Need to run ldconfig before building eSpeak because ...... ¯\_(ツ)_/¯
RUN ldconfig && \
    install-from-source https://github.com/espeak-ng/espeak-ng/archive/refs/tags/1.51.tar.gz \
    --no-toolchain \
    --autotools-force-regenerate

# mio memory mapping library
RUN install-from-source https://github.com/mandreyel/mio/archive/3f86a95c0784d73ce6815237ec33ed25f233b643.tar.gz \
    --patch https://patch-diff.githubusercontent.com/raw/mandreyel/mio/pull/93.patch

# zstr header to have file streams that do zlib compression
RUN install-from-source https://raw.githubusercontent.com/mateidavid/zstr/v1.0.6/src/zstr.hpp \
    --header-path include/zstr && \
    install-from-source https://raw.githubusercontent.com/mateidavid/zstr/v1.0.6/src/strict_fstream.hpp \
    --header-path include/zstr

# Visual Mesh
RUN install-from-source https://github.com/NUbots/VisualMesh/archive/64c49fce3a17bbee73339eaee556f47a80b93c40.tar.gz \
    -DBUILD_TENSORFLOW_OP=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_OPENCL_ENGINE=ON

# JSON for modern C++ (https://github.com/nlohmann/json)
RUN install-from-source https://github.com/nlohmann/json/releases/download/v3.10.5/json.hpp

# TinyXML
RUN install-from-source https://github.com/leethomason/tinyxml2/archive/refs/tags/9.0.0.tar.gz

# NLopt
RUN install-from-source https://github.com/stevengj/nlopt/archive/refs/tags/v2.7.1.tar.gz

# tinyrobotics
RUN install-from-source https://github.com/Tom0Brien/tinyrobotics/archive/refs/tags/0.0.1.tar.gz \
    -DBUILD_TESTS=OFF

# lame
RUN install-from-source https://downloads.sourceforge.net/lame/lame-3.100.tar.gz

#######################################
### ADD NEW PROGRAMS/LIBRARIES HERE ###
#######################################

# Install python libraries
RUN pip install \
    pygit2==1.13.3 \
    xxhash \
    pylint \
    termcolor \
    stringcase \
    Pillow \
    protobuf==4.21.2 \
    mycroft-mimic3-tts \
    tqdm

# Install tools needed for building individual modules as well as development tools
RUN install-package \
    arm-none-eabi-gcc \
    arm-none-eabi-newlib \
    doxygen

# Install formatters as last step so if they change the image is easy to update
RUN pip install --upgrade \
    cmake-format==0.6.13 \
    isort==5.10.1 \
    black==22.6.0 \
    licenseheaders==0.8.8

RUN wget -nv https://nodejs.org/dist/v20.13.1/node-v20.13.1-linux-x64.tar.xz -O - \
    | tar --extract --xz --strip-components=1 --directory /usr --file - \
    node-v20.13.1-linux-x64/bin/ node-v20.13.1-linux-x64/include/ node-v20.13.1-linux-x64/lib/
RUN npm install -g \
    yarn@1.22.19 \
    prettier@3.4.1 \
    eslint@8.57.0

# Enable all the driver capabilities so we can use OpenCL in the container
ENV NVIDIA_DRIVER_CAPABILITIES=all
RUN install-package clinfo

# Copy the nvidia OpenCL vendorfile into the container so we can also use OpenCL on nvidia GPUs
COPY --chown=root:root etc/OpenCL/vendors/nvidia.icd /etc/OpenCL/vendors/nvidia.icd

# Re-run ldconfig to make sure the database is up to date
RUN ldconfig

##############################
### SETUP THE USER ACCOUNT ###
##############################

# Setup the sudo so it can be run without a password
COPY etc/sudoers.d/user /etc/sudoers.d/user
RUN chmod 440 /etc/sudoers.d/user

# Create the user, and setup sudo so no password is required
ARG user_uid=1000
ARG user_gid=$user_uid
ARG dialout_gid=20

# useradd -G is not sufficient to add the nubots user to the dialout
# group as docker manages to lose supplementary groups when changing
# to the nubots user.
# Also need to add "--group-add dialout" to the docker run command in run.py
# Add user to uucp as well so people can directly test with arch/ubuntu/pop devices
RUN groupadd --gid $dialout_gid dialout \
    && groupadd --gid $user_gid nubots \
    && useradd --uid $user_uid --gid nubots -G audio,dialout,uucp -m nubots

USER nubots

# Without this git gets very angry and confused about the state of the user
RUN git config --global --add safe.directory '*'

# Copy over GDB alias'
COPY --chown=nubots:nubots home/nubots/.gdbinit /home/nubots/.gdbinit

# Setup the locations where we will mount our folders
RUN install -d -m 0755 /home/nubots/NUbots \
    && install -d -m 0777 /home/nubots/build \
    && install -d -m 0777 /home/nubots/NUbots/nusight2/node_modules

# Configure the cache directories
ENV CCACHE_DIR /home/nubots/build/ccache

WORKDIR /home/nubots/NUbots
