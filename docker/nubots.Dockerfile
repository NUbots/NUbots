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
    llvm \
    clang \
    libva \
    libpciaccess \
    && rm -rf /var/cache \
    && sed "s/^\(PREFIXES\s=\s\)\[\([^]]*\)\]/\1[\2, '\/usr\/local']/" -i /usr/lib/python3.7/site.py
RUN groupadd -r nubots && useradd --no-log-init -r -g nubots nubots

# Make sure /usr/local is checked for libraries and binaries
RUN mkdir -p "/etc/ld.so.conf.d" \
    && echo -e "/usr/local/lib\n/usr/local/lib64" | tee "/etc/ld.so.conf.d/usrlocal.conf" \
    && ldconfig
ENV PATH="/usr/local/bin:/usr/local/sbin:/usr/bin:/usr/sbin:/bin:/sbin"

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
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-meson-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-from-source-with-patches

# Add Intel OpenCL ICD file
RUN mkdir -p "/etc/OpenCL/vendors" && \
    echo "/usr/local/lib/intel-opencl/libigdrcl.so" | tee "/etc/OpenCL/vendors/intel.icd"

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

# Intel Compute Runtime (OpenCL) and Intel Media Driver
RUN install-from-source-with-patches https://github.com/KhronosGroup/SPIRV-LLVM-Translator/archive/v8.0.1-2.tar.gz \
    https://raw.githubusercontent.com/intel/opencl-clang/94af090661d7c953c516c97a25ed053c744a0737/patches/spirv/0001-Update-LowerOpenCL-pass-to-handle-new-blocks-represn.patch \
    https://raw.githubusercontent.com/intel/opencl-clang/94af090661d7c953c516c97a25ed053c744a0737/patches/spirv/0002-Remove-extra-semicolon.patch \
    -- \
    -Wno-dev
RUN install-from-source-with-patches https://github.com/intel/opencl-clang/archive/v8.0.1.tar.gz \
    https://github.com/intel/opencl-clang/commit/a6e69b30a6a2c925254784be808ae3171ecd75ea.patch \
    https://github.com/intel/opencl-clang/commit/94af090661d7c953c516c97a25ed053c744a0737.patch \
    -- \
    -DLLVMSPIRV_INCLUDED_IN_LLVM=OFF \
    -DSPIRV_TRANSLATOR_DIR=/usr/local \
    -DLLVM_NO_DEAD_STRIP=ON \
    -Wno-dev
COPY --chown=nubots:nubots package/IGC/*.patch /var/tmp/
RUN PREFIX=${PREFIX:-"/usr/local"} \
    && BUILD_FOLDER="/var/tmp/build" \
    && RELEASE_CFLAGS="-O3 -DNDEBUG" \
    && RELEASE_CXXFLAGS="${RELEASE_CFLAGS}" \
    && EXTRA_CFLAGS="-fPIC" \
    && EXTRA_CXXFLAGS="${EXTRA_CFLAGS}" \
    && . /usr/local/toolchain.sh \
    && mkdir -p "${BUILD_FOLDER}" \
    && cd "${BUILD_FOLDER}" \
    && wget https://github.com/intel/intel-graphics-compiler/archive/igc-1.0.10.tar.gz \
    && ARCHIVE_FILE=$(find . -type f | head -n 1) \
    && tar xf "${ARCHIVE_FILE}" \
    && echo "Configuring using cmake" \
    && CMAKELISTS_FILE=$(find -type f -name 'CMakeLists.txt' -printf '%d\t%P\n' | sort -nk1 | cut -f2- | head -n 1) \
    && cd $(dirname ${CMAKELISTS_FILE}) \
    && patch -Np1 -i /var/tmp/Intrinsics.py.patch \
    && patch -Np1 -i /var/tmp/resource_embedder.py.patch \
    && patch -Np1 -i /var/tmp/sip.py.patch \
    && echo "Configuring using cmake file ${CMAKELISTS_FILE}" \
    && mkdir -p build \
    && cd build \
    && cmake .. \
    -DIGC_OPTION__ARCHITECTURE_TARGET='Linux64' \
    -DIGC_PREFERRED_LLVM_VERSION='8.0.0' \
    -Wno-dev \
    -DCMAKE_BUILD_TYPE="Release" \
    -DCMAKE_C_FLAGS_RELEASE="${EXTRA_CFLAGS} ${CFLAGS}" \
    -DCMAKE_CXX_FLAGS_RELEASE="${EXTRA_CXXFLAGS} ${CXXFLAGS}" \
    -DCMAKE_INSTALL_PREFIX:PATH="${PREFIX}" \
    -DPKG_CONFIG_USE_CMAKE_PREFIX_PATH=ON \
    -DCMAKE_PREFIX_PATH:PATH="${PREFIX}" \
    -DCMAKE_INSTALL_LIBDIR=lib \
    && make -j$(nproc) \
    && make install \
    && rm -rf "${BUILD_FOLDER}"
RUN install-from-source https://github.com/intel/gmmlib/archive/intel-gmmlib-19.2.3.tar.gz \
    -DRUN_TEST_SUITE=OFF \
    -Wno-dev
RUN PREFIX=${PREFIX:-"/usr/local"} \
    && BUILD_FOLDER="/var/tmp/build" \
    && RELEASE_CFLAGS="-O3 -DNDEBUG" \
    && RELEASE_CXXFLAGS="${RELEASE_CFLAGS}" \
    && EXTRA_CFLAGS="-fPIC" \
    && EXTRA_CXXFLAGS="${EXTRA_CFLAGS}" \
    && . /usr/local/toolchain.sh \
    && mkdir -p "${BUILD_FOLDER}" \
    && cd "${BUILD_FOLDER}" \
    && wget https://github.com/intel/compute-runtime/archive/19.32.13826/intel-compute-runtime-19.32.13826.tar.gz \
    && ARCHIVE_FILE=$(find . -type f | head -n 1) \
    && tar xf "${ARCHIVE_FILE}" \
    && echo "Configuring using cmake" \
    && CMAKELISTS_FILE=$(find -type f -name 'CMakeLists.txt' -printf '%d\t%P\n' | sort -nk1 | cut -f2- | head -n 1) \
    && cd $(dirname ${CMAKELISTS_FILE}) \
    && echo "Configuring using cmake file ${CMAKELISTS_FILE}" \
    && mkdir -p build \
    && cd build \
    && cmake .. \
    -DNEO_DRIVER_VERSION=19.32.13826 \
    -DSKIP_ALL_ULT=ON \
    -DSKIP_UNIT_TESTS=ON \
    -DCMAKE_INSTALL_LIBDIR=lib \
    -DIGDRCL__IGC_LIBRARY_PATH="/usr/local/lib" \
    -DCMAKE_BUILD_TYPE="Release" \
    -DCMAKE_C_FLAGS_RELEASE="${EXTRA_CFLAGS} ${CFLAGS}" \
    -DCMAKE_CXX_FLAGS_RELEASE="${EXTRA_CXXFLAGS} ${CXXFLAGS}" \
    -DCMAKE_INSTALL_PREFIX:PATH="${PREFIX}" \
    -DPKG_CONFIG_USE_CMAKE_PREFIX_PATH=ON \
    -DCMAKE_PREFIX_PATH:PATH="${PREFIX}" \
    -DCMAKE_INSTALL_LIBDIR=lib \
    && make -j$(nproc) \
    && mkdir -p "/usr/local/lib/intel-opencl" \
    && mkdir -p "/usr/local/bin" \
    && cp "bin/libigdrcl.so" "/usr/local/lib/intel-opencl/libigdrcl.so" \
    && cp "bin/ocloc" "/usr/local/bin/ocloc" \
    && rm -rf "${BUILD_FOLDER}"
RUN install-from-source https://github.com/intel/media-driver/archive/intel-media-19.2.1.tar.gz \
    -DINSTALL_DRIVER_SYSCONF=OFF \
    -Wno-dev6


# Setup pip to install to /usr/local and have python find packages there
ENV PYTHONPATH=/usr/local/lib/python3.7/site-packages
COPY --chown=root:root files/pip.conf /etc/pip.conf

# Install python libraries
RUN pip install \
    protobuf==3.9.1 \
    stringcase

# http://www.fftw.org/fftw-3.3.6-pl2.tar.gz
# http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz
# https://dl.bintray.com/boostorg/release/1.64.0/source/boost_1_64_0.tar.gz

# Go to where we will mount the NUbots volume
WORKDIR /home/nubots/NUbots
