##
## MIT License
##
## Copyright (c) 2026 NUbots
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
FROM ubuntu:jammy-20260322.1

ARG platform=generic

# Prevent interactive prompts during apt-get installs (e.g. tzdata, keyboard-configuration)
ENV DEBIAN_FRONTEND=noninteractive

# Add a script that installs packages
COPY usr/local/bin/install-package /usr/local/bin/install-package

# Install base packages needed for building general toolchain
# If you have a tool that's needed for a specific module install it before that module
RUN install-package \
    wget \
    sudo \
    python3 \
    python3-pip \
    python3-tqdm \
    python3-venv \
    python3-requests \
    build-essential \
    clang \
    ccache \
    ninja-build \
    unzip \
    cmake \
    cmake-curses-gui \
    meson \
    git \
    zlib1g-dev \
    openssh-client \
    rsync \
    gdb \
    colordiff \
    parallel \
    vim \
    nano \
    libmagic1 \
    libmagic-dev

COPY usr/local/bin/container_setup.sh /usr/local/bin/container_setup.sh
RUN /usr/local/bin/container_setup.sh ${platform}

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
    python3 /usr/local/toolchain/generate_generic_toolchain.py --prefix /usr

##############################################
### ADD NEW SYSTEM PROGRAMS/LIBRARIES HERE ###
##############################################

################################################
#  _____           _      _           _        #
# |_   _|__   ___ | | ___| |__   __ _(_)_ __   #
#   | |/ _ \ / _ \| |/ __| '_ \ / _` | | '_ \  #
#   | | (_) | (_) | | (__| | | | (_| | | | | | #
#   |_|\___/ \___/|_|\___|_| |_|\__,_|_|_| |_| #
################################################
ARG prefix=/usr/local

# Create a python virtual environment at ${PREFIX} to decouple python packages from the system
RUN python3 -m venv ${prefix}

# Install python dependencies for the install-from-source script
RUN pip3 install \
    python-magic==0.4.27 \
    requests==2.32.3 \
    termcolor==2.5.0 \
    tqdm==4.62.3

# Generate toolchain files for the current platform
RUN --mount=type=bind,source=usr/local/toolchain,target=/usr/local/toolchain \
    python3 /usr/local/toolchain/generate_${platform}_toolchain.py --prefix ${prefix}


# We install ncurses rather than building it as it's used by many of the build tools and they don't like it when it changes
RUN install-package -p ${platform} libncurses-dev libncurses6 libtinfo6 libc6-dev && \
    if [ "${platform}" != "generic" ]; then \
    ln -sf libncursesw.so.6 /l4t/targetfs/usr/lib/aarch64-linux-gnu/libncursesw.so && \
    ln -sf libtinfo.so.6    /l4t/targetfs/usr/lib/aarch64-linux-gnu/libtinfo.so; \
    fi

RUN pip3 install pyyaml mako

# Install OpenCL C Headers
RUN install-from-source https://github.com/KhronosGroup/OpenCL-Headers/archive/refs/tags/v2022.05.18.tar.gz \
    --prefix /l4t/targetfs/${prefix} \
    --toolchain-dir /usr/local


# OpenCL loader library
RUN install-package ruby autotools-dev automake libtool
RUN install-from-source https://github.com/OCL-dev/ocl-icd/archive/refs/tags/v2.3.1.tar.gz \
    --prefix /l4t/targetfs/${prefix} \
    --toolchain-dir /usr/local

RUN if [ "${platform}" != "generic" ]; then \
    ln -sf libm.so.6      /l4t/targetfs/usr/lib/aarch64-linux-gnu/libm.so && \
    ln -sf libpthread.so.0 /l4t/targetfs/usr/lib/aarch64-linux-gnu/libpthread.so && \
    ln -sf libdl.so.2      /l4t/targetfs/usr/lib/aarch64-linux-gnu/libdl.so && \
    ln -sf librt.so.1      /l4t/targetfs/usr/lib/aarch64-linux-gnu/librt.so; \
    fi

# OpenCV
RUN install-from-source https://github.com/opencv/opencv/archive/refs/tags/4.9.0.tar.gz

RUN wget -nv https://github.com/protocolbuffers/protobuf/releases/download/v21.4/protoc-21.4-linux-x86_64.zip -O protoc-21.4-linux-x86_64.zip && \
    unzip protoc-21.4-linux-x86_64.zip -d /usr/ && \
    rm protoc-21.4-linux-x86_64.zip

# Protobuf
RUN install-from-source https://github.com/protocolbuffers/protobuf/releases/download/v21.4/protobuf-cpp-3.21.4.tar.gz \
    -Dprotobuf_PROTOC_EXECUTABLE="/usr/bin/protoc" \
    -Dprotobuf_BUILD_TESTS=OFF \
    -Dprotobuf_BUILD_LIBPROTOC=ON \
    -DBUILD_SHARED_LIBS=OFF

# Install natively for Jetson as well, and also install protoc for host
RUN if [ "${platform}" != "generic" ]; then \
    wget -nv https://github.com/protocolbuffers/protobuf/releases/download/v21.4/protoc-21.4-linux-aarch_64.zip -O protoc-21.4-linux-aarch_64.zip && \
    unzip protoc-21.4-linux-aarch_64.zip -d /l4t/targetfs/usr/ && \
    rm protoc-21.4-linux-aarch_64.zip && \
    install-from-source https://github.com/protocolbuffers/protobuf/releases/download/v21.4/protobuf-cpp-3.21.4.tar.gz \
    --toolchain-dir /usr \
    -Dprotobuf_PROTOC_EXECUTABLE="/usr/bin/protoc" \
    -Dprotobuf_BUILD_TESTS=OFF \
    -Dprotobuf_BUILD_LIBPROTOC=ON \
    -DBUILD_SHARED_LIBS=OFF; \
    fi

# OpenVino - Dependencies
RUN install-package opencl-c-headers \
    shellcheck \
    scons \
    pkgconf \
    patchelf \
    fdupes \
    ca-certificates \
    file \
    rapidjson-dev \
    libsnappy-dev

# Clone OpenVINO repository
RUN git clone https://github.com/openvinotoolkit/openvino.git --branch 2024.6.0 && \
    cd openvino && \
    git submodule update --init --recursive

# Build OpenVINO
WORKDIR /openvino
RUN mkdir build && cd build && \
    cmake .. -DBUILD_TESTING:BOOL='OFF' \
    -DCMAKE_BUILD_TYPE:STRING='Release' \
    -DCMAKE_TOOLCHAIN_FILE=${prefix}/toolchain.cmake \
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
    -DENABLE_INTEL_GPU:BOOL='OFF' \
    -DENABLE_INTEL_CPU:BOOL='OFF' \
    -DENABLE_CPPLINT:BOOL='OFF' \
    -DCMAKE_CXX_STANDARD='17' \
    -Wno-dev \
    -GNinja && \
    ninja -j$(nproc) && ninja -j$(nproc) install

# Move OpenVino libraries to our library path
RUN ARCH=$([ "${platform}" = "orinnx" ] && echo "aarch64" || echo "intel64") && \
    LIB_PATH=$([ "${platform}" = "orinnx" ] && echo "/l4t/targetfs/usr/local/lib" || echo "/usr/local/lib") && \
    cp /openvino/bin/${ARCH}/Release/* ${LIB_PATH}/ -r

# CUDA Toolkit
COPY usr/local/bin/install-cuda.sh /usr/local/bin/install-cuda.sh
RUN /usr/local/bin/install-cuda.sh ${platform}

RUN install-package -p ${platform} \
    cuda-cudart-12-5 \
    cuda-cudart-dev-12-5 \
    cuda-compat-12-5 \
    cuda-driver-dev-12-5 \
    cuda-nvcc-12-5 \
    cuda-crt-12-5 \
    libcublas-12-5 \
    libcublas-dev-12-5 \
    libnvjitlink-12-5 \
    libnvjitlink-dev-12-5 \
    cuda-nvrtc-12-5 \
    cuda-nvrtc-dev-12-5

# TensorRT - install binary packages from NVIDIA CUDA repository
# NvInfer.h, NvOnnxParser.h, libnvinfer.so, libnvonnxparsers.so
RUN install-package -p ${platform} \
    libnvinfer-dev \
    libnvonnxparsers-dev \
    libnvinfer-plugin-dev \
    libnvinfer-lean-dev \
    libnvinfer-vc-plugin-dev

# Eigen3
RUN install-from-source https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz

# tcmalloc
RUN install-from-source https://github.com/gperftools/gperftools/releases/download/gperftools-2.10/gperftools-2.10.tar.gz \
    --build-system autotools \
    --prefix /l4t/targetfs/${prefix} \
    --toolchain-dir /usr/local \
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
RUN install-from-source https://github.com/catchorg/Catch2/archive/refs/tags/v3.8.0.tar.gz \
    -DCATCH_CONFIG_CONSOLE_WIDTH=120 \
    -DCATCH_CONFIG_CPP11_TO_STRING=ON \
    -DCATCH_CONFIG_CPP17_UNCAUGHT_EXCEPTIONS=ON \
    -DCATCH_CONFIG_CPP17_STRING_VIEW=ON \
    -DCATCH_CONFIG_CPP17_VARIANT=ON \
    -DCATCH_CONFIG_CPP17_OPTIONAL=ON \
    -DCATCH_CONFIG_CPP17_BYTE=ON

# LibUV
RUN install-from-source https://github.com/libuv/libuv/archive/refs/tags/v1.44.2.tar.gz \
    -Dlibuv_buildtests=OFF \
    -DBUILD_TESTING=OFF

# NUClear!
RUN install-from-source https://github.com/Fastcode/NUClear/archive/925dca0f31484a7df64fd335de5a6c9335483c7f.tar.gz \
    -DBUILD_TESTS=OFF

# Backtrace
RUN install-from-source https://github.com/ianlancetaylor/libbacktrace/archive/8602fda64e78f1f46563220f2ee9f7e70819c51d.tar.gz \
    --prefix /l4t/targetfs/${prefix} \
    --toolchain-dir /usr/local \
    --without-system-libunwind \
    --enable-shared \
    --enable-static

# Alsa and espeak
RUN install-from-source https://github.com/alsa-project/alsa-lib/archive/refs/tags/v1.2.7.2.tar.gz \
    --prefix /l4t/targetfs/${prefix} \
    --toolchain-dir /usr/local \
    --without-debug
RUN install-package -p ${platform} alsa-utils

# mio memory mapping library
RUN install-from-source https://github.com/mandreyel/mio/archive/3f86a95c0784d73ce6815237ec33ed25f233b643.tar.gz \
    --patch https://patch-diff.githubusercontent.com/raw/mandreyel/mio/pull/93.patch

# zstr header to have file streams that do zlib compression
RUN INCLUDE_PREFIX=$([ "${platform}" = "orinnx" ] && echo "/l4t/targetfs/usr/local" || echo "/usr/local") && \
    install-from-source https://raw.githubusercontent.com/mateidavid/zstr/v1.0.6/src/zstr.hpp \
    --prefix ${INCLUDE_PREFIX} --no-toolchain \
    --header-path include/zstr && \
    install-from-source https://raw.githubusercontent.com/mateidavid/zstr/v1.0.6/src/strict_fstream.hpp \
    --prefix ${INCLUDE_PREFIX} --no-toolchain \
    --header-path include/zstr

# Visual Mesh
RUN install-from-source https://github.com/NUbots/VisualMesh/archive/64c49fce3a17bbee73339eaee556f47a80b93c40.tar.gz \
    -DBUILD_TENSORFLOW_OP=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_OPENCL_ENGINE=ON

# JSON for modern C++ (https://github.com/nlohmann/json)
RUN INCLUDE_PREFIX=$([ "${platform}" = "orinnx" ] && echo "/l4t/targetfs/usr/local" || echo "/usr/local") && \
    install-from-source https://github.com/nlohmann/json/releases/download/v3.10.5/json.hpp \
    --prefix ${INCLUDE_PREFIX} --no-toolchain

# TinyXML
RUN install-from-source https://github.com/leethomason/tinyxml2/archive/refs/tags/9.0.0.tar.gz

# NLopt
RUN install-from-source https://github.com/stevengj/nlopt/archive/refs/tags/v2.7.1.tar.gz

# tinyrobotics
RUN install-from-source https://github.com/Tom0Brien/tinyrobotics/archive/refs/tags/0.0.1.tar.gz \
    -DBUILD_TESTS=OFF

# lame
RUN install-from-source https://downloads.sourceforge.net/lame/lame-3.100.tar.gz \
    --prefix /l4t/targetfs/${prefix} \
    --toolchain-dir /usr/local

# Nanopb for generating messages for NUSense
RUN install-from-source https://github.com/nanopb/nanopb/archive/refs/tags/0.4.9.1.tar.gz

# Install Booster Robotics SDK

RUN install-package curl lsb-release

RUN curl -L https://github.com/BoosterRobotics/booster_robotics_sdk/archive/324946e7c885de6cd6d37220eca64ed4e0f8d418.tar.gz \
    -o /tmp/booster_robotics_sdk.tar.gz && \
    cd /tmp && tar -xzf booster_robotics_sdk.tar.gz && \
    mv booster_robotics_sdk-324946e7c885de6cd6d37220eca64ed4e0f8d418 booster_robotics_sdk && \
    rm /tmp/booster_robotics_sdk.tar.gz

WORKDIR /tmp/booster_robotics_sdk
RUN curl -L https://gist.githubusercontent.com/ZeoNyph/69e4ec80b66cc083191522de7eb4f43b/raw/ef9393748b5a8797fb8ef08d9eaa56ade76d7547/install_script.patch \
    -o install_script.patch && \
    patch -p1 < install_script.patch

RUN BOOSTER_ARG=$([ "${platform}" = "orinnx" ] && echo "aarch64" || echo "x86_64") && \
    /tmp/booster_robotics_sdk/install.sh ${BOOSTER_ARG:+$BOOSTER_ARG} && \
    rm -rf /tmp/booster_robotics_sdk

RUN install-package -p ${platform} libusb-1.0-0-dev libcurl4-openssl-dev zlib1g-dev libboost-dev

#######################################
### ADD NEW PROGRAMS/LIBRARIES HERE ###
#######################################

# Install tools needed for building individual modules as well as development tools
RUN install-package \
    doxygen \
    gcc-12 \
    g++-12


# Node tools
RUN wget -nv https://nodejs.org/dist/v20.13.1/node-v20.13.1-linux-x64.tar.xz -O - \
    | tar --extract --xz --strip-components=1 --directory /usr --file - \
    node-v20.13.1-linux-x64/bin/ node-v20.13.1-linux-x64/include/ node-v20.13.1-linux-x64/lib/
RUN npm install -g \
    yarn@1.22.19 \
    prettier@3.4.1 \
    eslint@8.57.0

# Install uv for python package management and set it to use the build directory for storage
RUN pip3 install uv==0.7.2

# When in the docker image uv will use a separate volume for the cache and venv
# Virtual environments are on the same volume as the uv cache to allow hard linking for ~speed~
ENV UV_CACHE_DIR=/home/nubots/python/uv-cache
ENV UV_PROJECT_ENVIRONMENT=/home/nubots/python/.venv

# Enable all the driver capabilities so we can use OpenCL in the container
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
RUN install-package clinfo

# Copy the nvidia OpenCL vendorfile into the container so we can also use OpenCL on nvidia GPUs
COPY --chown=root:root etc/OpenCL/vendors/nvidia.icd /etc/OpenCL/vendors/nvidia.icd

# Re-run ldconfig to make sure the database is up to date
RUN ln -sf /l4t/targetfs/usr/local/include/opencv4/opencv2 /l4t/targetfs/usr/local/include/opencv2
RUN ldconfig

##############################
### SETUP THE USER ACCOUNT ###
##############################

# Setup the sudo so it can be run without a password
COPY etc/sudoers.d/user /etc/sudoers.d/user
RUN chmod 440 /etc/sudoers.d/user

# Create the user and setup sudo so no password is required
ARG user_uid=1000
ARG user_gid=$user_uid
ARG render_gid=110

# useradd -G is not sufficient to add the nubots user to the dialout
# group as docker manages to lose supplementary groups when changing
# to the nubots user.
# Also need to add "--group-add dialout" to the docker run command in run.py
# Add user to uucp as well so people can directly test with arch/ubuntu/pop devices
RUN groupadd --gid $render_gid render_host \
    && groupadd --gid $user_gid nubots \
    && useradd --uid $user_uid --gid nubots -G audio,dialout,uucp,video,render_host -m nubots

USER nubots

# Without this git gets very angry and confused about the state of the user
RUN git config --global --add safe.directory '*'

# Copy over GDB alias'
COPY --chown=nubots:nubots home/nubots/.gdbinit /home/nubots/.gdbinit


# Setup the locations where we will mount our folders
RUN install -d -m 0755 /home/nubots/NUbots \
    && install -d -m 0777 /home/nubots/build \
    && install -d -m 0777 /home/nubots/python \
    && install -d -m 0777 /home/nubots/NUbots/nusight2/node_modules

# Configure the cache directories
ENV CCACHE_DIR=/home/nubots/build/ccache

WORKDIR /home/nubots/NUbots
