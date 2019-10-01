##############################################
#   ___  ____    ____       _                #
#  / _ \/ ___|  / ___|  ___| |_ _   _ _ __   #
# | | | \___ \  \___ \ / _ \ __| | | | '_ \  #
# | |_| |___) |  ___) |  __/ |_| |_| | |_) | #
#  \___/|____/  |____/ \___|\__|\__,_| .__/  #
#                                    |_|     #
##############################################
FROM archlinux/base:latest

# Create the nubots user, and setup sudo so no password is required
RUN useradd --no-log-init -m -U nubots
COPY --chown=root:root etc/sudoers.d/user /etc/sudoers.d/user

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
RUN sed "s/^\(PREFIXES\s=\s\)\[\([^]]*\)\]/\1[\2, '\/usr\/local']/" -i /usr/lib/python3.7/site.py \
    && mkdir -p /usr/local/lib/python3.7/site-packages
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
COPY --chown=nubots:nubots usr/local/bin/install-from-source /usr/local/bin/install-from-source
RUN ln -s /usr/local/bin/install-from-source /usr/local/bin/install-header-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-cmake-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-autotools-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-bjam-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-make-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-meson-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-python-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-from-source-with-patches

# Build llvm/clang and install to /usr so we can use them from cross-compiling
RUN sudo PREFIX="/usr" install-cmake-from-source \
    https://github.com/llvm/llvm-project/releases/download/llvmorg-8.0.1/llvm-8.0.1.src.tar.xz
COPY --chown=nubots:nubots usr/local/package/clang/install-from-source /usr/local/package/clang.sh
RUN  sudo PREFIX="/usr" /usr/local/package/clang.sh \
    https://github.com/llvm/llvm-project/releases/download/llvmorg-8.0.1/cfe-8.0.1.src.tar.xz

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

# Intel Compute Runtime (OpenCL), libva and Intel Media Driver (VAAPI)
RUN install-cmake-from-source https://github.com/llvm/llvm-project/releases/download/llvmorg-8.0.1/llvm-8.0.1.src.tar.xz
RUN install-from-source https://github.com/llvm/llvm-project/releases/download/llvmorg-8.0.1/cfe-8.0.1.src.tar.xz \
    -DLLVM_TABLEGEN=/usr/bin/llvm-tblgen \
    -DCLANG_TABLEGEN=/usr/bin/clang-tblgen
RUN install-from-source https://xorg.freedesktop.org/releases/individual/lib/libpciaccess-0.16.tar.bz2
RUN install-meson-from-source https://dri.freedesktop.org/libdrm/libdrm-2.4.99.tar.bz2 \
    -Dudev=false \
    -Dvalgrind=false
RUN install-from-source https://github.com/intel/libva/releases/download/2.5.0/libva-2.5.0.tar.bz2
RUN install-from-source-with-patches https://github.com/KhronosGroup/SPIRV-LLVM-Translator/archive/v8.0.1-2.tar.gz \
    https://raw.githubusercontent.com/intel/opencl-clang/94af090661d7c953c516c97a25ed053c744a0737/patches/spirv/0001-Update-LowerOpenCL-pass-to-handle-new-blocks-represn.patch \
    https://raw.githubusercontent.com/intel/opencl-clang/94af090661d7c953c516c97a25ed053c744a0737/patches/spirv/0002-Remove-extra-semicolon.patch \
    --
COPY --chown=nubots:nubots usr/local/package/opencl-clang/install-from-source /usr/local/package/opencl-clang.sh
RUN /usr/local/package/opencl-clang.sh https://github.com/intel/opencl-clang/archive/v8.0.1.tar.gz
COPY --chown=nubots:nubots usr/local/package/intel-graphics-compiler/python3.patch /usr/local/package/intel-graphics-compiler/
RUN install-from-source-with-patches https://github.com/intel/intel-graphics-compiler/archive/igc-1.0.10.tar.gz \
    /usr/local/package/intel-graphics-compiler/python3.patch \
    -- \
    -DIGC_OPTION__ARCHITECTURE_TARGET='Linux64' \
    -DIGC_PREFERRED_LLVM_VERSION='8.0.0'
RUN install-from-source https://github.com/intel/gmmlib/archive/intel-gmmlib-19.2.4.tar.gz \
    -DRUN_TEST_SUITE=OFF
COPY --chown=root:root etc/OpenCL/vendors/intel.icd /etc/OpenCL/vendors/intel.icd
COPY --chown=nubots:nubots usr/local/package/intel-compute-runtime/install-from-source /usr/local/package/intel-compute-runtime.sh
RUN /usr/local/package/intel-compute-runtime.sh \
    https://github.com/intel/compute-runtime/archive/19.34.13959.tar.gz \
    -DNEO_DRIVER_VERSION=19.32.13826 \
    -DSKIP_ALL_ULT=ON \
    -DSKIP_UNIT_TESTS=ON \
    -DIGDRCL__IGC_LIBRARY_PATH="/usr/local/lib"
COPY --chown=nubots:nubots usr/local/package/opencl-headers/install-from-source /usr/local/package/opencl-headers.sh
RUN /usr/local/package/opencl-headers.sh https://github.com/KhronosGroup/OpenCL-Headers/archive/master.tar.gz
RUN install-from-source https://github.com/KhronosGroup/OpenCL-CLHPP/archive/master.tar.gz \
    -DOPENCL_INCLUDE_DIR=/usr/local/include/CL \
    -DOPENCL_LIB_DIR=/usr/local/lib \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF
RUN install-package ruby
COPY --chown=nubots:nubots usr/local/package/ocl-icd/install-from-source /usr/local/package/ocl-icd.sh
RUN /usr/local/package/ocl-icd.sh https://github.com/OCL-dev/ocl-icd/archive/v2.2.12.tar.gz
RUN install-from-source https://github.com/intel/media-driver/archive/intel-media-19.2.1.tar.gz \
    -DINSTALL_DRIVER_SYSCONF=OFF \
    -DLIBVA_DRIVERS_PATH="/usr/local/lib/dri" \
    -DMEDIA_RUN_TEST_SUITE=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_KERNELS=ON \
    -DENABLE_KERNELS=ON \
    -DENABLE_NONFREE_KERNELS=ON \
    -DBUILD_TYPE=Release

# OpenBLAS
RUN install-package gcc-fortran
COPY --chown=nubots:nubots usr/local/package/openblas/${platform}.sh usr/local/package/openblas.sh
RUN /usr/local/package/openblas.sh https://github.com/xianyi/OpenBLAS/archive/v0.3.7.tar.gz

# Armadillo
RUN install-cmake-from-source https://downloads.sourceforge.net/project/arma/armadillo-9.700.2.tar.xz \
    -DDETECT_HDF5=OFF \
    -DBUILD_SHARED_LIBS=ON
COPY --chown=nubots:nubots usr/local/include/armadillo_bits/config.hpp /usr/local/include/armadillo_bits/config.hpp

# Eigen3
RUN install-from-source http://bitbucket.org/eigen/eigen/get/3.3.7.tar.bz2

# tcmalloc
RUN install-from-source \
    https://github.com/gperftools/gperftools/releases/download/gperftools-2.7/gperftools-2.7.tar.gz \
    --with-tcmalloc-pagesize=64 \
    --enable-minimal

# Protobuf
RUN install-package protobuf
RUN install-from-source https://github.com/google/protobuf/releases/download/v3.7.0/protobuf-cpp-3.7.0.tar.gz \
    --with-zlib=/usr/local \
    --with-protoc=/usr/bin/protoc
RUN PROTOC=/usr/bin/protoc install-python-from-source \
    https://github.com/google/protobuf/releases/download/v3.7.0/protobuf-python-3.7.0.tar.gz --cpp_implementation

# Libjpeg
RUN install-package yasm
RUN install-from-source https://github.com/libjpeg-turbo/libjpeg-turbo/archive/2.0.2.tar.gz \
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
RUN install-from-source https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.2.tar.gz \
    -DYAML_CPP_BUILD_TESTS=OFF \
    -DBUILD_SHARED_LIBS=OFF

# fmt formatting library
RUN  install-from-source https://github.com/fmtlib/fmt/archive/6.0.0.tar.gz \
    -DFMT_DOC=OFF \
    -DFMT_TEST=OFF

# Catch unit testing library
RUN install-header-from-source https://github.com/catchorg/Catch2/releases/download/v2.9.2/catch.hpp

# Aravis
RUN install-from-source http://xmlsoft.org/sources/libxml2-2.9.7.tar.gz --with-zlib=/usr/local --without-python
RUN install-from-source https://github.com/libffi/libffi/archive/v3.3-rc0.tar.gz
RUN install-from-source https://mirrors.edge.kernel.org/pub/linux/utils/util-linux/v2.34/util-linux-2.34.tar.xz \
    --disable-all-programs \
    --enable-libblkid \
    --enable-libmount \
    --enable-libuuid
RUN install-from-source https://gitlab.gnome.org/GNOME/glib/-/archive/2.61.2/glib-2.61.2.tar.gz \
    -Ddefault_library=both \
    -Dinternal_pcre=true \
    && cp /usr/local/lib/glib-2.0/include/glibconfig.h /usr/local/include/glibconfig.h
RUN install-meson-from-source https://github.com/AravisProject/aravis/archive/ARAVIS_0_6_4.tar.gz \
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

# libbacktrace
RUN install-from-source https://github.com/ianlancetaylor/libbacktrace/archive/master.tar.gz \
    --without-system-libunwind \
    --enable-shared \
    --enable-static

# Install python libraries
RUN pip install \
    cmake-format \
    black \
    termcolor \
    stringcase \
    Pillow

# Install tools needed for building individual modules as well as development tools
RUN install-package \
    arm-none-eabi-gcc \
    arm-none-eabi-newlib \
    openssh \
    rsync \
    gdb \
    valgrind \
    colordiff \
    parallel

# Copy ssh keys over to the system
RUN install -d -m 0755 -o nubots -g nubots /home/nubots/.ssh
COPY --chown=nubots:nubots home/nubots/.ssh/id_rsa /home/nubots/.ssh/id_rsa
COPY --chown=nubots:nubots home/nubots/.ssh/id_rsa.pub /home/nubots/.ssh/id_rsa.pub
COPY --chown=nubots:nubots home/nubots/.ssh/config /home/nubots/.ssh/config
RUN chmod 600 /home/nubots/.ssh/id_rsa /home/nubots/.ssh/config

# Setup the locations where we will mount our folders
RUN mkdir -p /home/nubots/NUbots
RUN mkdir -p /home/nubots/build
WORKDIR /home/nubots/NUbots
