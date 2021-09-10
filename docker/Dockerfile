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
FROM archlinux:base-20210110.0.13332 as base

# ↓↓↓ UPDATE MIRRORS HERE ↓↓↓
RUN echo -e "Server=http://host.docker.internal:7878/\$repo/os/\$arch\nServer=https://archive.archlinux.org/repos/2021/01/10/\$repo/os/\$arch" | tee /etc/pacman.d/mirrorlist

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
    nodejs-lts-fermium \
    npm \
    yarn \
    git \
    ncurses

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

# zlib
RUN install-from-source https://www.zlib.net/zlib-1.2.11.tar.gz

# LLVM and Clang
RUN BUILD_FILE_DIR='llvm' install-cmake-from-source https://github.com/llvm/llvm-project/archive/llvmorg-11.0.1.tar.gz \
    -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra" \
    -DLLVM_ENABLE_SPHINX=OFF \
    -DLLVM_BUILD_LLVM_DYLIB=ON \
    -DLLVM_LINK_LLVM_DYLIB=ON

# SPIRV LLVM Translator
RUN install-cmake-from-source https://github.com/KhronosGroup/SPIRV-LLVM-Translator/archive/v11.0.0.tar.gz \
    -DLLVM_INCLUDE_TESTS=OFF

# opencl-clang
COPY usr/local/package/opencl-clang.sh /usr/local/package/opencl-clang.sh
RUN /usr/local/package/opencl-clang.sh https://github.com/intel/opencl-clang/archive/v11.0.0.tar.gz

# intel graphics compiler
COPY usr/local/package/intel-graphics-compiler.sh /usr/local/package/intel-graphics-compiler.sh
RUN /usr/local/package/intel-graphics-compiler.sh https://github.com/intel/intel-graphics-compiler/archive/igc-1.0.5435.tar.gz \
    -DIGC_OPTION__ARCHITECTURE_TARGET='Linux64' \
    -DIGC_PREFERRED_LLVM_VERSION='11.0.0' \
    -DVC_INTRINSICS_SRC="../../vc-intrinsics" \
    -DINSTALL_SPIRVDLL=0

# intel compute runtime dependencies
# gmmlib
RUN install-cmake-from-source https://github.com/intel/gmmlib/archive/intel-gmmlib-20.4.1.tar.gz \
    -DRUN_TEST_SUITE=OFF
# libpciaccess and libdrm help communicate with graphics hardware
RUN install-from-source https://xorg.freedesktop.org/releases/individual/lib/libpciaccess-0.16.tar.bz2
RUN install-meson-from-source https://dri.freedesktop.org/libdrm/libdrm-2.4.104.tar.xz \
    -Dudev=false \
    -Dvalgrind=false
# libva
RUN install-from-source https://github.com/intel/libva/releases/download/2.10.0/libva-2.10.0.tar.bz2 \
    -DENABLE_VA_MESSAGING=ON

# intel media driver
RUN install-cmake-from-source https://github.com/intel/media-driver/archive/intel-media-20.4.5.tar.gz \
    -DINSTALL_DRIVER_SYSCONF=OFF \
    -DLIBVA_DRIVERS_PATH="/usr/local/lib/dri" \
    -DMEDIA_RUN_TEST_SUITE=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_KERNELS=ON \
    -DENABLE_KERNELS=ON \
    -DENABLE_NONFREE_KERNELS=ON

# Install intel compute runtime (OpenCL implementation)
COPY --chown=root:root etc/OpenCL/vendors/intel.icd /etc/OpenCL/vendors/intel.icd
COPY usr/local/package/intel-compute-runtime.sh /usr/local/package/intel-compute-runtime.sh
RUN /usr/local/package/intel-compute-runtime.sh https://github.com/intel/compute-runtime/archive/20.48.18558.tar.gz \
    -DSKIP_UNIT_TESTS=ON \
    -DIGC_DIR=/usr/local

# Install OpenCL C and C++ headers
COPY usr/local/package/opencl-headers.sh /usr/local/package/opencl-headers.sh
RUN /usr/local/package/opencl-headers.sh https://github.com/KhronosGroup/OpenCL-Headers/archive/v2020.12.18.tar.gz
RUN install-from-source https://github.com/KhronosGroup/OpenCL-CLHPP/archive/master.tar.gz \
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

# libusb - a dependency of aravis
RUN install-from-source https://github.com/libusb/libusb/archive/v1.0.24.tar.gz
# Aravis
RUN install-from-source http://xmlsoft.org/sources/libxml2-2.9.10.tar.gz --with-zlib=/usr/local --without-python
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
RUN install-cmake-from-source https://github.com/libuv/libuv/archive/v1.34.1.tar.gz \
    -Dlibuv_buildtests=OFF \
    -DBUILD_TESTING=OFF

# NUClear!
RUN install-from-source https://github.com/Fastcode/NUClear/archive/153ccdb9a13dbc88f3c448e07a3048d67235116e.tar.gz \
    -DBUILD_TESTS=OFF

# Visual Mesh
RUN install-from-source https://github.com/Fastcode/VisualMesh/archive/9df04eab3b08d2dca1d955a5bfbdbeea75f11d6b.tar.gz \
    -DBUILD_TENSORFLOW_OP=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_OPENCL_ENGINE=ON

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
RUN install-from-source https://github.com/mandreyel/mio/archive/master.tar.gz

# zstr header to have file streams that do zlib compression
RUN mkdir -p /usr/local/include/zstr \
    && wget https://raw.githubusercontent.com/mateidavid/zstr/946ab409380e90e4a46c88c2da4f645a7ffaa51d/src/zstr.hpp -O /usr/local/include/zstr.hpp \
    && wget https://raw.githubusercontent.com/mateidavid/zstr/946ab409380e90e4a46c88c2da4f645a7ffaa51d/src/strict_fstream.hpp -O /usr/local/include/strict_fstream.hpp \
    && wget https://raw.githubusercontent.com/mateidavid/zstr/946ab409380e90e4a46c88c2da4f645a7ffaa51d/src/zstr_make_unique_polyfill.h -O /usr/local/include/zstr_make_unique_polyfill.h

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
    nano \
    clinfo

# Install formatters as last step so if they change the image is easy to update
RUN pip install --upgrade \
    cmake-format==0.6.13 \
    isort==5.7.0 \
    black==20.8b1

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

#############################################
#               Voice2json                  #
#############################################

FROM base AS openfst

RUN git clone https://aur.archlinux.org/openfst.git /home/nubots/openfst
WORKDIR /home/nubots/openfst
# Enable configure build options for openfst
RUN sed "34a \ \ OPTIONS+=' --enable-static'" -i PKGBUILD
RUN sed "35a \ \ OPTIONS+=' --enable-shared'" -i PKGBUILD
RUN makepkg || true
RUN test -f openfst-1.7.9-1-x86_64.pkg.tar.zst

FROM base AS kaldi-openfst

RUN install-package python2 subversion unzip sox gcc-fortran gcc-libs
RUN git clone https://aur.archlinux.org/kaldi-openfst.git /home/nubots/kaldi-openfst
WORKDIR /home/nubots/kaldi-openfst
# Set to a better(?) version number.
RUN sed '/^pkgver=/s/1.6.7/1.7.2/' -i PKGBUILD
RUN makepkg || true
RUN test -f kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst

FROM base AS kaldi

RUN git clone https://aur.archlinux.org/kaldi.git /home/nubots/kaldi
WORKDIR /home/nubots/kaldi
RUN install-package lapack python2 cblas
RUN sed '/^pkgver=/s/5.5.r9114.d6198906f/5.5.r9187.d53b62fd4/' -i PKGBUILD
COPY --from=kaldi-openfst /home/nubots/kaldi-openfst/kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst /home/nubots/kaldi/
RUN sudo pacman -U --noconfirm kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst
RUN makepkg || true
RUN test -f kaldi-5.5.r9195.3b8a97d85-1-x86_64.pkg.tar.zst

FROM base AS ngram

RUN git clone https://aur.archlinux.org/opengrm-ngram.git /home/nubots/opengrm-ngram
WORKDIR /home/nubots/opengrm-ngram
COPY --from=openfst /home/nubots/openfst/openfst-1.7.9-1-x86_64.pkg.tar.zst /home/nubots/opengrm-ngram
RUN sudo pacman -U --noconfirm openfst-1.7.9-1-x86_64.pkg.tar.zst

# Set the package name because it changed recently at opengrm.org
RUN sed '/^pkgname/s/=.*/=ngram/' -i PKGBUILD
# Set version number.
RUN sed '/^pkgver/s/=.*/=1.3.11/' -i PKGBUILD
# Set source url. The old one didn't exist.
RUN sed '/^source/s/=.*/=("http:\/\/opengrm.org\/twiki\/pub\/GRM\/NGramDownload\/${pkgname}-${pkgver}.tar.gz")/' -i PKGBUILD
# Set the hash value. This value wasn't obtained securely (i.e. I downloaded insecure file then ran sha256sum)
RUN sed "/^md5sums=/s/.*/sha256sums=('05d417b57751dc7eca43b13d5f50eebe9d2338851bc47a854bb9318b91649773')/" -i PKGBUILD
RUN makepkg || true
RUN test -f ngram-1.3.11-1-x86_64.pkg.tar.zst

FROM base AS phonetisaurus

RUN git clone https://aur.archlinux.org/python-pybindgen.git /home/nubots/python-pybindgen
WORKDIR /home/nubots/python-pybindgen
RUN makepkg
RUN sudo pacman -U --noconfirm python-pybindgen-0.20.1-1-x86_64.pkg.tar.zst

RUN git clone https://aur.archlinux.org/phonetisaurus-git.git /home/nubots/phonetisaurus-git
WORKDIR /home/nubots/phonetisaurus-git
COPY --from=openfst /home/nubots/openfst/openfst-1.7.9-1-x86_64.pkg.tar.zst /home/nubots/phonetisaurus-git/
RUN sudo pacman -U --noconfirm openfst-1.7.9-1-x86_64.pkg.tar.zst

# Set the version number.
RUN sed -i PKGBUILD -e '/^pkgver/s/=.*$/=r171.23f182f/'
# Add configuration options to ./configure
RUN sed -i PKGBUILD -e '30a\\ \t\t--with-openfst-includes=/usr/include/fst \\'
RUN sed -i PKGBUILD -e '31a\\ \t\t--with-openfst-libs=/usr/lib \\'
RUN makepkg || true
RUN test -f phonetisaurus-git-r171.23f182f-1-x86_64.pkg.tar.zst

FROM base

WORKDIR /home/nubots/packages/
COPY --from=kaldi-openfst       /home/nubots/kaldi-openfst/kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=kaldi               /home/nubots/kaldi/kaldi-5.5.r9195.3b8a97d85-1-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=openfst             /home/nubots/openfst/openfst-1.7.9-1-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=ngram               /home/nubots/opengrm-ngram/ngram-1.3.11-1-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=phonetisaurus       /home/nubots/python-pybindgen/python-pybindgen-0.20.1-1-x86_64.pkg.tar.zst /home/nubots/packages/
COPY --from=phonetisaurus       /home/nubots/phonetisaurus-git/phonetisaurus-git-r171.23f182f-1-x86_64.pkg.tar.zst /home/nubots/packages/

WORKDIR /home/nubots/packages
RUN sudo pacman -U --noconfirm \
    kaldi-openfst-1.7.2-2-x86_64.pkg.tar.zst \
    kaldi-5.5.r9195.3b8a97d85-1-x86_64.pkg.tar.zst \
    openfst-1.7.9-1-x86_64.pkg.tar.zst \
    ngram-1.3.11-1-x86_64.pkg.tar.zst \
    python-pybindgen-0.20.1-1-x86_64.pkg.tar.zst \
    phonetisaurus-git-r171.23f182f-1-x86_64.pkg.tar.zst

# voice2json
RUN install-package python-wheel swig alsa-lib jack portaudio sox gcc-fortran jq espeak
RUN git clone https://github.com/synesthesiam/voice2json /home/nubots/voice2json
WORKDIR /home/nubots/voice2json
RUN touch mod.sed
RUN echo "/@ENABLE_POCKETSPHINX@/s//no/" >> mod.sed
RUN echo "/@ENABLE_KALDI@/s//yes/" >> mod.sed
RUN echo "/@ENABLE_DEEPSPEECH@/s//no/" >> mod.sed
RUN echo "/@PACKAGE_NAME@/s//voice2json/" >> mod.sed
RUN echo "/@PACKAGE_VERSION@/s//0.2.0/" >> mod.sed
RUN echo "/@PACKAGE_BUGREPORT@/s//mike@rhasspy.org/" >> mod.sed
RUN sed -f mod.sed setup.py.in > setup.py
RUN sudo python setup.py install

# Tell voice2json where kaldi is located.
ENV KALDI_DIR=/opt/kaldi
# Make sure we search these paths too:
#   echo /opt/kaldi/src/*bin | sed 's/ /:/g'
ENV PATH="$PATH:/opt/kaldi/src/bin:/opt/kaldi/src/chainbin:/opt/kaldi/src/cudadecoderbin:/opt/kaldi/src/cudafeatbin:/opt/kaldi/src/featbin:/opt/kaldi/src/fgmmbin:/opt/kaldi/src/fstbin:/opt/kaldi/src/gmmbin:/opt/kaldi/src/ivectorbin:/opt/kaldi/src/kwsbin:/opt/kaldi/src/latbin:/opt/kaldi/src/lmbin:/opt/kaldi/src/nnet2bin:/opt/kaldi/src/nnet3bin:/opt/kaldi/src/nnetbin:/opt/kaldi/src/online2bin:/opt/kaldi/src/rnnlmbin:/opt/kaldi/src/sgmm2bin:/opt/kaldi/src/tfrnnlmbin"
# Some scripts try to use python23... (make_lexicon_fst.py)
RUN sudo ln /usr/bin/python3 /usr/bin/python23
# Tell voice2json where voice2json is in case you are not within /home/nubots/voice2json.
ENV voice2json_dir=/home/nubots/voice2json
# Download the kaldi speech to text model.
RUN python -m voice2json download-profile
COPY --chown=nubots:nubots voice2json/sentences.ini /home/nubots/.local/share/voice2json/en-us_kaldi-zamia/
COPY --chown=nubots:nubots voice2json/slots/ /home/nubots/.local/share/voice2json/en-us_kaldi-zamia/slots/
RUN python -m voice2json train-profile

WORKDIR /home/nubots/NUbots
