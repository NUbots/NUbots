##############################################
#   ___  ____    ____       _                #
#  / _ \/ ___|  / ___|  ___| |_ _   _ _ __   #
# | | | \___ \  \___ \ / _ \ __| | | | '_ \  #
# | |_| |___) |  ___) |  __/ |_| |_| | |_) | #
#  \___/|____/  |____/ \___|\__|\__,_| .__/  #
#                                    |_|     #
##############################################
FROM archlinux:20200106

# Arch recommends running this as the first command of the docker image
RUN pacman -Syu --noconfirm

# Create the nubots user, and setup sudo so no password is required
RUN useradd --no-log-init -m -U nubots
COPY --chown=root:root etc/sudoers.d/user /etc/sudoers.d/user
RUN chmod 440 /etc/sudoers.d/user

# Add a script that installs packages
COPY --chown=nubots:nubots usr/local/bin/install-package /usr/local/bin/install-package

# Install base packages needed for building general toolchain
# If you have a tool that's needed for a specific module install it before that module
RUN install-package \
    sudo \
    wget \
    python \
    python-pip \
    base-devel \
    ninja \
    cmake \
    meson \
    git

# Get python to look in /usr/local for packages
RUN echo $(python -c "import site; print(site.getsitepackages()[0].replace('/usr', '/usr/local'))") \
    > $(python -c "import site; print(site.getsitepackages()[0])")/local.pth
COPY --chown=root:root etc/pip.conf /etc/pip.conf

# Make sure /usr/local is checked for libraries and binaries
COPY --chown=root:root etc/ld.so.conf.d/usrlocal.conf /etc/ld.so.conf.d/usrlocal.conf
RUN ldconfig

# Setup /usr/local owned by nubots and swap to the nubots user
RUN chown -R nubots:nubots /usr/local
USER nubots

# Make a symlink from /usr/local/lib to /usr/local/lib64 so library install location is irrelevant
RUN cd /usr/local && ln -sf lib lib64

# Generate toolchain files for the generic platform
COPY --chown=nubots:nubots usr/local/toolchain/generate_toolchains.py /usr/local/generate_toolchains.py
COPY --chown=nubots:nubots usr/local/toolchain/generate_generic_toolchain.py /usr/local/generate_toolchain.py
RUN sudo python /usr/local/generate_toolchain.py --prefix /usr

# Copy over a tool to install simple standard conforming libraries from source
COPY --chown=nubots:nubots usr/local/bin/download-and-extract /usr/local/bin/download-and-extract
COPY --chown=nubots:nubots usr/local/bin/install-from-source /usr/local/bin/install-from-source
RUN ln -s /usr/local/bin/install-from-source /usr/local/bin/install-header-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-cmake-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-autotools-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-bjam-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-make-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-meson-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-python-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-from-source-with-patches

# Generate toolchain files for generic
COPY --chown=nubots:nubots usr/local/toolchain/generate_generic_toolchain.py /usr/local/generate_toolchain.py
RUN sudo python /usr/local/generate_toolchain.py --prefix /usr

#############################################
### ADD NEW SYTEM PROGRAMS/LIBRARIES HERE ###
#############################################

################################################
#  _____           _      _           _        #
# |_   _|__   ___ | | ___| |__   __ _(_)_ __   #
#   | |/ _ \ / _ \| |/ __| '_ \ / _` | | '_ \  #
#   | | (_) | (_) | | (__| | | | (_| | | | | | #
#   |_|\___/ \___/|_|\___|_| |_|\__,_|_|_| |_| #
################################################
ARG platform=generic

# Generate toolchain files for the current platform
COPY --chown=nubots:nubots usr/local/toolchain/generate_${platform}_toolchain.py /usr/local/generate_toolchain.py
RUN python /usr/local/generate_toolchain.py --prefix /usr/local

# zlib
RUN install-from-source https://www.zlib.net/zlib-1.2.11.tar.gz

# LLVM and Clang
RUN BUILD_FILE_DIR='llvm' install-cmake-from-source \
    https://github.com/llvm/llvm-project/archive/llvmorg-9.0.1.tar.gz \
    -DLLVM_ENABLE_PROJECTS="clang"

# Libpci access and libdrm for communicating with graphics hardware
RUN install-from-source https://xorg.freedesktop.org/releases/individual/lib/libpciaccess-0.16.tar.bz2
RUN install-meson-from-source https://dri.freedesktop.org/libdrm/libdrm-2.4.100.tar.bz2 \
    -Dudev=false \
    -Dvalgrind=false

# Intel graphics compiler
RUN install-cmake-from-source https://github.com/KhronosGroup/SPIRV-LLVM-Translator/archive/v9.0.0-1.tar.gz
COPY --chown=nubots:nubots usr/local/package/opencl-clang.sh /usr/local/package/opencl-clang.sh
RUN /usr/local/package/opencl-clang.sh https://github.com/intel/opencl-clang/archive/v9.0.0.tar.gz
RUN install-from-source https://github.com/intel/intel-graphics-compiler/archive/igc-1.0.3041.tar.gz \
    -DIGC_OPTION__ARCHITECTURE_TARGET='Linux64' \
    -DIGC_PREFERRED_LLVM_VERSION='9.0.1'
RUN install-from-source https://github.com/intel/gmmlib/archive/intel-gmmlib-19.4.1.tar.gz \
    -DRUN_TEST_SUITE=OFF

# Libva and the iHD driver for hardware compression
RUN install-from-source https://github.com/intel/libva/releases/download/2.6.0/libva-2.6.0.tar.bz2
RUN install-from-source https://github.com/intel/media-driver/archive/intel-media-19.4.0r.tar.gz \
    -DINSTALL_DRIVER_SYSCONF=OFF \
    -DLIBVA_DRIVERS_PATH="/usr/local/lib/dri" \
    -DMEDIA_RUN_TEST_SUITE=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_KERNELS=ON \
    -DENABLE_KERNELS=ON \
    -DENABLE_NONFREE_KERNELS=ON \
    -DBUILD_TYPE=Release

# Install intel compute runtime (OpenCL implementation)
COPY --chown=root:root etc/OpenCL/vendors/intel.icd /etc/OpenCL/vendors/intel.icd
COPY --chown=nubots:nubots usr/local/package/intel-compute-runtime.sh /usr/local/package/intel-compute-runtime.sh
RUN /usr/local/package/intel-compute-runtime.sh https://github.com/intel/compute-runtime/archive/20.01.15264.tar.gz \
    -DNEO_DRIVER_VERSION=19.40.14409 \
    -DSKIP_ALL_ULT=ON \
    -DSKIP_UNIT_TESTS=ON \
    -DIGDRCL__IGC_LIBRARY_PATH="/usr/local/lib"

# Install OpenCL C and C++ headers
COPY --chown=nubots:nubots usr/local/package/opencl-headers.sh /usr/local/package/opencl-headers.sh
RUN /usr/local/package/opencl-headers.sh https://github.com/KhronosGroup/OpenCL-Headers/archive/master.tar.gz
RUN install-from-source https://github.com/KhronosGroup/OpenCL-CLHPP/archive/master.tar.gz \
    -DOPENCL_INCLUDE_DIR=/usr/local/include/CL \
    -DOPENCL_LIB_DIR=/usr/local/lib \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF

# OpenCL loader library
RUN install-package ruby
RUN install-from-source https://github.com/OCL-dev/ocl-icd/archive/v2.2.12.tar.gz

# OpenBLAS
RUN install-package gcc-fortran
COPY --chown=nubots:nubots usr/local/package/openblas/${platform}.sh usr/local/package/openblas.sh
RUN /usr/local/package/openblas.sh https://github.com/xianyi/OpenBLAS/archive/v0.3.7.tar.gz

# Armadillo
RUN install-cmake-from-source https://downloads.sourceforge.net/project/arma/armadillo-9.800.3.tar.xz \
    -DDETECT_HDF5=OFF \
    -DBUILD_SHARED_LIBS=ON
COPY --chown=nubots:nubots usr/local/include/armadillo_bits/config.hpp /usr/local/include/armadillo_bits/config.hpp

# Eigen3
COPY --chown=nubots:nubots usr/local/package/eigen3/deprecated-copy.patch usr/local/package/deprecated-copy.patch
COPY --chown=nubots:nubots usr/local/package/eigen3/cxx17-overalign.patch usr/local/package/cxx17-overalign.patch
RUN install-from-source-with-patches https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2 \
    /usr/local/package/deprecated-copy.patch \
    /usr/local/package/cxx17-overalign.patch --

# tcmalloc
RUN install-from-source \
    https://github.com/gperftools/gperftools/releases/download/gperftools-2.7/gperftools-2.7.tar.gz \
    --with-tcmalloc-pagesize=64 \
    --enable-minimal

# Protobuf
RUN sudo install-package protobuf
RUN install-autotools-from-source https://github.com/protocolbuffers/protobuf/releases/download/v3.11.2/protobuf-cpp-3.11.2.tar.gz \
    --with-zlib=/usr/local \
    --with-protoc=/usr/sbin/protoc

# Libjpeg
RUN install-package yasm
RUN install-from-source https://github.com/libjpeg-turbo/libjpeg-turbo/archive/2.0.4.tar.gz \
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
RUN install-from-source https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.3.tar.gz \
    -DYAML_CPP_BUILD_TESTS=OFF \
    -DBUILD_SHARED_LIBS=OFF

# fmt formatting library
RUN  install-from-source https://github.com/fmtlib/fmt/archive/6.1.2.tar.gz \
    -DFMT_DOC=OFF \
    -DFMT_TEST=OFF

# Catch unit testing library
RUN install-header-from-source https://github.com/catchorg/Catch2/releases/download/v2.11.1/catch.hpp

# Aravis
RUN install-from-source http://xmlsoft.org/sources/libxml2-2.9.10.tar.gz --with-zlib=/usr/local --without-python
RUN install-from-source https://github.com/libffi/libffi/releases/download/v3.3/libffi-3.3.tar.gz
RUN install-from-source https://mirrors.edge.kernel.org/pub/linux/utils/util-linux/v2.34/util-linux-2.34.tar.xz \
    --disable-all-programs \
    --enable-libblkid \
    --enable-libmount \
    --enable-libuuid
RUN install-from-source https://gitlab.gnome.org/GNOME/glib/-/archive/2.62.4/glib-2.62.4.tar.bz2 \
    -Ddefault_library=both \
    -Dinternal_pcre=true \
    && cp /usr/local/lib/glib-2.0/include/glibconfig.h /usr/local/include/glibconfig.h
RUN install-meson-from-source https://github.com/AravisProject/aravis/archive/ARAVIS_0_7_3.tar.gz \
    -Ddefault_library=both \
    -Dviewer=false \
    -Dgst-plugin=false \
    -Dusb=true \
    -Ddocumentation=false \
    -Dintrospection=false

# LibUV
RUN install-cmake-from-source https://github.com/libuv/libuv/archive/v1.34.1.tar.gz \
    -Dlibuv_buildtests=OFF \
    -DBUILD_TESTING=OFF

# NUClear!
RUN install-from-source https://github.com/Fastcode/NUClear/archive/master.tar.gz \
    -DBUILD_TESTS=OFF

# Backtrace
RUN install-from-source https://github.com/ianlancetaylor/libbacktrace/archive/master.tar.gz \
    --without-system-libunwind \
    --enable-shared \
    --enable-static

# Alsa and espeak
RUN install-from-source ftp://ftp.alsa-project.org/pub/lib/alsa-lib-1.2.1.2.tar.bz2 \
    --without-debug
RUN install-from-source https://github.com/espeak-ng/pcaudiolib/archive/1.1.tar.gz
COPY --chown=nubots:nubots usr/local/package/espeak.sh /usr/local/package/espeak.sh
RUN /usr/local/package/espeak.sh https://github.com/espeak-ng/espeak-ng/releases/download/1.50/espeak-ng-1.50.tgz

# Install python libraries
RUN pip install \
    cmake-format \
    black \
    pylint \
    isort \
    termcolor \
    stringcase \
    Pillow \
    protobuf==3.11.2 \
    tqdm

# Install tools needed for building individual modules as well as development tools
RUN install-package \
    arm-none-eabi-gcc \
    arm-none-eabi-newlib \
    openssh \
    rsync \
    gdb \
    valgrind \
    colordiff \
    parallel \
    vim \
    nano

#######################################
### ADD NEW PROGRAMS/LIBRARIES HERE ###
#######################################

# Copy ssh keys over to the system
RUN install -d -m 0755 -o nubots -g nubots /home/nubots/.ssh
COPY --chown=nubots:nubots home/nubots/.ssh/id_rsa /home/nubots/.ssh/id_rsa
COPY --chown=nubots:nubots home/nubots/.ssh/id_rsa.pub /home/nubots/.ssh/id_rsa.pub
COPY --chown=nubots:nubots home/nubots/.ssh/config /home/nubots/.ssh/config
RUN chmod 600 /home/nubots/.ssh/id_rsa /home/nubots/.ssh/config

# Setup the locations where we will mount our folders
RUN install -d -m 0755 -o nubots -g nubots /home/nubots/NUbots
RUN install -d -m 0755 -o nubots -g nubots /home/nubots/build
WORKDIR /home/nubots/NUbots
