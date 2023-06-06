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
FROM archlinux:base-20210829.0.32635

# ↓↓↓ UPDATE MIRRORS HERE ↓↓↓
RUN echo -e "Server=https://archive.archlinux.org/repos/2021/08/29/\$repo/os/\$arch" | tee /etc/pacman.d/mirrorlist
# If you ever update the mirror date, search the repository for UPDATE MIRRORS HERE to find all the places where the URL
# needs to be altered.

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
    base-devel \
    ninja \
    cmake \
    meson \
    git \
    ncurses \
    zlib

# Get python to look in /usr/local for packages
RUN echo $(python -c "import site; print(site.getsitepackages()[0].replace('/usr', '/usr/local'))") \
    > $(python -c "import site; print(site.getsitepackages()[0])")/local.pth
COPY etc/xdg/pip/pip.conf /etc/xdg/pip/pip.conf

# Make sure /usr/local is checked for libraries and binaries
COPY etc/ld.so.conf.d/usrlocal.conf /etc/ld.so.conf.d/usrlocal.conf
RUN ldconfig

# Make a symlink from /usr/local/lib to /usr/local/lib64 so library install location is irrelevant
RUN cd /usr/local && ln -sf lib lib64

# Generate toolchain files for the generic platform
COPY usr/local/toolchain/generate_toolchains.py /usr/local/generate_toolchains.py
COPY usr/local/toolchain/generate_generic_toolchain.py /usr/local/generate_toolchain.py
RUN python /usr/local/generate_toolchain.py --prefix /usr

# Copy over a tool to install simple standard conforming libraries from source
COPY usr/local/bin/download-and-extract /usr/local/bin/download-and-extract
COPY usr/local/bin/install-from-source /usr/local/bin/install-from-source
RUN ln -s /usr/local/bin/install-from-source /usr/local/bin/install-header-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-cmake-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-autotools-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-bjam-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-make-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-meson-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-python-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-from-source-with-patches

# Generate toolchain files for generic
COPY usr/local/toolchain/generate_generic_toolchain.py /usr/local/generate_toolchain.py
RUN python /usr/local/generate_toolchain.py --prefix /usr

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
COPY usr/local/toolchain/generate_${platform}_toolchain.py /usr/local/generate_toolchain.py
RUN python /usr/local/generate_toolchain.py --prefix /usr/local

# LLVM and Clang
RUN BUILD_FILE_DIR='llvm' install-cmake-from-source https://github.com/llvm/llvm-project/archive/llvmorg-12.0.1.tar.gz \
    -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra;lld" \
    -DLLVM_ENABLE_SPHINX=OFF \
    -DLLVM_BUILD_LLVM_DYLIB=ON \
    -DLLVM_LINK_LLVM_DYLIB=ON

# SPIRV LLVM Translator
RUN install-cmake-from-source https://github.com/KhronosGroup/SPIRV-LLVM-Translator/archive/v12.0.0.tar.gz \
    -DLLVM_INCLUDE_TESTS=OFF

# opencl-clang
COPY usr/local/package/opencl-clang.sh /usr/local/package/opencl-clang.sh
RUN /usr/local/package/opencl-clang.sh https://github.com/intel/opencl-clang/archive/v12.0.0.tar.gz

# intel graphics compiler
COPY usr/local/package/intel-graphics-compiler.sh /usr/local/package/intel-graphics-compiler.sh
RUN /usr/local/package/intel-graphics-compiler.sh https://github.com/intel/intel-graphics-compiler/archive/igc-1.0.8173.tar.gz \
    -DIGC_OPTION__ARCHITECTURE_TARGET='Linux64' \
    -DIGC_OPTION__LLVM_PREFERRED_VERSION='12.0.1' \
    -DVC_INTRINSICS_SRC="../../vc-intrinsics" \
    -DIGC_OPTION__LLVM_MODE=Prebuilds \
    -DCMAKE_CXX_FLAGS="-Wno-deprecated-declarations" \
    -DINSTALL_SPIRVDLL=0 \
    -DINSTALL_GENX_IR=ON

# intel compute runtime dependencies
# gmmlib
RUN install-cmake-from-source https://github.com/intel/gmmlib/archive/intel-gmmlib-21.2.2.tar.gz \
    -DRUN_TEST_SUITE=OFF
# libpciaccess and libdrm help communicate with graphics hardware
RUN install-from-source https://xorg.freedesktop.org/releases/individual/lib/libpciaccess-0.16.tar.bz2
RUN install-meson-from-source https://dri.freedesktop.org/libdrm/libdrm-2.4.104.tar.xz \
    -Dudev=false \
    -Dvalgrind=false

# libva
# Needs -Dwith_x11: see https://github.com/intel/media-driver/issues/899
# -Dwith_x11 requires x11 to be installed
RUN install-package xorg-server
RUN install-from-source https://github.com/intel/libva/releases/download/2.12.0/libva-2.12.0.tar.bz2 \
    -DENABLE_VA_MESSAGING=ON \
    -Dwith_x11=yes

# intel media driver
RUN install-cmake-from-source https://github.com/intel/media-driver/archive/intel-media-21.3.2.tar.gz \
    -DINSTALL_DRIVER_SYSCONF=OFF \
    -DLIBVA_DRIVERS_PATH="/usr/local/lib/dri" \
    -DMEDIA_RUN_TEST_SUITE=OFF \
    -DBUILD_KERNELS=ON \
    -DENABLE_KERNELS=ON \
    -DENABLE_NONFREE_KERNELS=ON \
    -DMEDIA_BUILD_FATAL_WARNINGS=OFF

# Install intel compute runtime (OpenCL implementation)
COPY --chown=root:root etc/OpenCL/vendors/intel.icd /etc/OpenCL/vendors/intel.icd
COPY usr/local/package/intel-compute-runtime.sh /usr/local/package/intel-compute-runtime.sh
RUN /usr/local/package/intel-compute-runtime.sh https://github.com/intel/compute-runtime/archive/21.31.20514.tar.gz \
    https://patch-diff.githubusercontent.com/raw/intel/compute-runtime/pull/550.patch -- \
    -DSKIP_UNIT_TESTS=ON \
    -DIGC_DIR=/usr/local

# Install OpenCL C and C++ headers
COPY usr/local/package/opencl-headers.sh /usr/local/package/opencl-headers.sh
RUN /usr/local/package/opencl-headers.sh https://github.com/KhronosGroup/OpenCL-Headers/archive/v2020.12.18.tar.gz
RUN install-from-source https://github.com/KhronosGroup/OpenCL-CLHPP/archive/refs/tags/v2.0.16.tar.gz \
    -DOPENCL_INCLUDE_DIR=/usr/local/include/CL \
    -DOPENCL_LIB_DIR=/usr/local/lib \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF

# OpenCL loader library
RUN install-package ruby
RUN install-from-source https://github.com/OCL-dev/ocl-icd/archive/v2.2.14.tar.gz

# OpenBLAS
RUN install-package gcc-fortran
COPY usr/local/package/openblas/${platform}.sh /usr/local/package/openblas.sh
RUN /usr/local/package/openblas.sh https://github.com/xianyi/OpenBLAS/archive/v0.3.13.tar.gz

# Eigen3
COPY usr/local/package/eigen3/deprecated-copy.patch /usr/local/package/deprecated-copy.patch
COPY usr/local/package/eigen3/cxx17-overalign.patch /usr/local/package/cxx17-overalign.patch
RUN install-from-source-with-patches https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2 \
    /usr/local/package/deprecated-copy.patch \
    /usr/local/package/cxx17-overalign.patch --

# tcmalloc
RUN install-from-source \
    https://github.com/gperftools/gperftools/releases/download/gperftools-2.7/gperftools-2.7.tar.gz \
    --with-tcmalloc-pagesize=64 \
    --enable-minimal

# Protobuf
RUN install-package protobuf
RUN install-autotools-from-source https://github.com/protocolbuffers/protobuf/releases/download/v3.17.3/protobuf-cpp-3.17.3.tar.gz \
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

# libusb - a dependency of aravis
RUN install-from-source https://github.com/libusb/libusb/archive/v1.0.24.tar.gz
# Aravis
RUN install-from-source http://xmlsoft.org/sources/libxml2-2.9.10.tar.gz --without-python
RUN install-from-source https://github.com/libffi/libffi/releases/download/v3.3/libffi-3.3.tar.gz
RUN install-from-source https://gitlab.gnome.org/GNOME/glib/-/archive/2.63.3/glib-2.63.3.tar.bz2 \
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
RUN install-cmake-from-source https://github.com/libuv/libuv/archive/v1.42.0.tar.gz \
    -Dlibuv_buildtests=OFF \
    -DBUILD_TESTING=OFF

# NUClear!
RUN install-from-source https://github.com/Fastcode/NUClear/archive/48aa321595c13ed3ba8781ffdc7f63fc70d6d427.tar.gz \
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
COPY usr/local/package/espeak.sh /usr/local/package/espeak.sh
RUN /usr/local/package/espeak.sh https://github.com/espeak-ng/espeak-ng/releases/download/1.50/espeak-ng-1.50.tgz

# mio memory mapping library
RUN install-from-source https://github.com/mandreyel/mio/archive/3f86a95c0784d73ce6815237ec33ed25f233b643.tar.gz

# zstr header to have file streams that do zlib compression
RUN mkdir -p /usr/local/include/zstr \
    && wget https://raw.githubusercontent.com/mateidavid/zstr/946ab409380e90e4a46c88c2da4f645a7ffaa51d/src/zstr.hpp -O /usr/local/include/zstr.hpp \
    && wget https://raw.githubusercontent.com/mateidavid/zstr/946ab409380e90e4a46c88c2da4f645a7ffaa51d/src/strict_fstream.hpp -O /usr/local/include/strict_fstream.hpp \
    && wget https://raw.githubusercontent.com/mateidavid/zstr/946ab409380e90e4a46c88c2da4f645a7ffaa51d/src/zstr_make_unique_polyfill.h -O /usr/local/include/zstr_make_unique_polyfill.h

# Visual Mesh
RUN install-from-source https://github.com/NUbots/VisualMesh/archive/64c49fce3a17bbee73339eaee556f47a80b93c40.tar.gz \
    -DBUILD_TENSORFLOW_OP=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_OPENCL_ENGINE=ON

# JSON for modern C++ (https://github.com/nlohmann/json)
RUN install-header-from-source https://github.com/nlohmann/json/releases/download/v3.10.5/json.hpp

#######################################
### ADD NEW PROGRAMS/LIBRARIES HERE ###
#######################################

# Install python libraries
RUN pip install \
    xxhash \
    pylint \
    termcolor \
    stringcase \
    Pillow \
    protobuf==3.17.3 \
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
    nano \
    clinfo \
    doxygen

# Install packages for nusight
RUN install-package \
    nodejs \
    npm \
    yarn

# Install formatters as last step so if they change the image is easy to update
RUN pip install --upgrade \
    cmake-format==0.6.13 \
    isort==5.7.0 \
    black==22.3.0

RUN npm install -g \
    prettier@2.7.1 \
    eslint@8.27.0

# Enable all the driver capabilities so we can use OpenCL in the container
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Copy the nvidia OpenCL vendorfile into the container so we can also use OpenCL on nvidia GPUs
COPY --chown=root:root etc/OpenCL/vendors/nvidia.icd /etc/OpenCL/vendors/nvidia.icd

##############################
### SETUP THE USER ACCOUNT ###
##############################

# Setup the sudo so it can be run without a password
COPY etc/sudoers.d/user /etc/sudoers.d/user
RUN chmod 440 /etc/sudoers.d/user

# Create the user, and setup sudo so no password is required
ARG user_uid=1000
ARG user_gid=$user_uid
RUN groupadd --gid $user_gid nubots \
    && useradd --uid $user_uid --gid $user_gid -m nubots
USER nubots

# Copy ssh keys over to the system
RUN install -d -m 0755 -o nubots -g nubots /home/nubots/.ssh
COPY --chown=nubots:nubots home/nubots/.ssh/id_rsa /home/nubots/.ssh/id_rsa
COPY --chown=nubots:nubots home/nubots/.ssh/id_rsa.pub /home/nubots/.ssh/id_rsa.pub
COPY --chown=nubots:nubots home/nubots/.ssh/config /home/nubots/.ssh/config
COPY --chown=nubots:nubots home/nubots/.gdbinit /home/nubots/.gdbinit
RUN chmod 600 /home/nubots/.ssh/id_rsa /home/nubots/.ssh/config

# Setup the locations where we will mount our folders
RUN install -d -m 0755 /home/nubots/NUbots
RUN install -d -m 0777 /home/nubots/build
WORKDIR /home/nubots/NUbots
