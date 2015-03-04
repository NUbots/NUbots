FROM 32bit/ubuntu:14.04
MAINTAINER Simon Hartcher "simon@simonhartcher.com"
MAINTAINER Trent Houliston "trent@houliston.me"
ENV HOSTNAME nubotsvm
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive

# Set our toolchain path
ENV TOOLCHAIN_PATH /nubots/toolchain
ENV CMAKE_PREFIX_PATH /nubots/toolchain

# Since apt-get is using the ubuntu extras repo you need keys or it will error
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 437D05B5 3E5C1192

# Makes errors with upstart not be so loud
RUN dpkg-divert --local --rename --add /sbin/initctl
RUN ln -s /bin/true /sbin/initctl

# Update our apt repository
RUN apt-get update

# Setup our repository for g++4.9
RUN apt-get -y install software-properties-common
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get update

# Get common helpful tools
RUN apt-get -y install vim
RUN apt-get -y install wget
RUN apt-get -y install curl
RUN apt-get -y install git
RUN apt-get -y install python

# Get our build tools
RUN apt-get -y install build-essential \
                       gfortran \
                       cmake \
                       cmake-curses-gui \
                       ninja-build

# Change our default linker to gold
RUN update-alternatives --install /usr/bin/ld ld /usr/bin/ld.bfd 10
RUN update-alternatives --install /usr/bin/ld ld /usr/bin/ld.gold 20

# Fix our ar, ranlib and nm tools to use the gcc versions
RUN echo '#/bin/bash'          > /usr/local/bin/ar && \
    echo '/usr/bin/ar $@ --plugin /usr/lib/gcc/i686-linux-gnu/4.9/liblto_plugin.so' >> /usr/local/bin/ar && \
    chmod +x /usr/local/bin/ar
RUN echo '#/bin/bash'              > /usr/local/bin/ranlib && \
    echo '/usr/bin/ranlib $@ --plugin /usr/lib/gcc/i686-linux-gnu/4.9/liblto_plugin.so' >> /usr/local/bin/ranlib && \
    chmod +x /usr/local/bin/ranlib
RUN echo '#/bin/bash'          > /usr/local/bin/nm && \
    echo '/usr/bin/nm $@ --plugin /usr/lib/gcc/i686-linux-gnu/4.9/liblto_plugin.so' >> /usr/local/bin/nm && \
    chmod +x /usr/local/bin/nm

# zlib
WORKDIR /tmp
RUN curl http://zlib.net/zlib-1.2.8.tar.gz | tar -xz
WORKDIR zlib-1.2.8
RUN CFLAGS="-fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    CXXFLAGS="-fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    LDFLAGS="-L$TOOLCHAIN_PATH/lib" \
    ./configure \
    --prefix=$TOOLCHAIN_PATH
RUN make test
RUN make install
WORKDIR /tmp
RUN rm -rf zlib-1.2.8

# libprotobuf + protobuf-compiler
WORKDIR /tmp
RUN curl -L https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz | tar -xz
WORKDIR protobuf-2.6.1/build
RUN CFLAGS="-fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    CXXFLAGS="-fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    LDFLAGS="-L$TOOLCHAIN_PATH/lib" \
    ../configure \
    --with-zlib \
    --prefix=$TOOLCHAIN_PATH
RUN make
RUN make install
WORKDIR /tmp
RUN rm -rf protobuf-2.6.1

# openpgm
WORKDIR /tmp
RUN curl -L https://openpgm.googlecode.com/files/libpgm-5.2.122.tar.gz | tar -xz
WORKDIR libpgm-5.2.122/openpgm/pgm/build
RUN CFLAGS="-fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    CXXFLAGS="-fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    LDFLAGS="-L$TOOLCHAIN_PATH/lib" \
    ../configure \
    --prefix=$TOOLCHAIN_PATH
RUN make
RUN make install
WORKDIR /tmp
RUN rm -rf libpgm-5.2.122

# libzmq4
WORKDIR /tmp
RUN curl -L http://download.zeromq.org/zeromq-4.0.5.tar.gz | tar -xz
WORKDIR zeromq-4.0.5/build
RUN CFLAGS="-fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    CXXFLAGS="-fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    LDFLAGS="-L$TOOLCHAIN_PATH/lib" \
    OpenPGM_CFLAGS="" \
    OpenPGM_LIBS="" \
    ../configure \
    --prefix=$TOOLCHAIN_PATH
RUN make
RUN make install
RUN wget https://raw.githubusercontent.com/zeromq/cppzmq/master/zmq.hpp -O "$TOOLCHAIN_PATH/include/zmq.hpp"
WORKDIR /tmp
RUN rm -rf zeromq-4.0.5

# NUClear
WORKDIR /tmp
RUN git clone -b OldDSL --depth 1 --single-branch https://github.com/FastCode/NUClear NUClear
WORKDIR /tmp/NUClear/build
RUN cmake .. -GNinja \
             -DCMAKE_C_FLAGS='-fuse-linker-plugin -flto -fno-fat-lto-objects' \
             -DCMAKE_CXX_FLAGS='-fuse-linker-plugin -flto -fno-fat-lto-objects' \
             -DNUCLEAR_BUILD_TESTS=OFF \
             -DCMAKE_INSTALL_PREFIX=$TOOLCHAIN_PATH
RUN ninja
RUN ninja install
WORKDIR /tmp
RUN rm -rf NUClear

# OpenBLAS (includes lapack)
WORKDIR /tmp
RUN curl -L https://github.com/xianyi/OpenBLAS/archive/v0.2.13.tar.gz | tar -xz
WORKDIR OpenBLAS-0.2.13
RUN TARGET=ATOM \
    USE_THREAD=1 \
    BINARY=32 \
    PREFIX=$TOOLCHAIN_PATH \
    COMMON_OPT='-fuse-linker-plugin -flto -fno-fat-lto-objects -O3' \
    FCOMMON_OPT='-fuse-linker-plugin -flto -fno-fat-lto-objects -O3' \
    make
RUN make PREFIX=$TOOLCHAIN_PATH install
WORKDIR /tmp
RUN rm -rf OpenBLAS-0.2.13

# Armadillo
WORKDIR /tmp
RUN curl -L http://sourceforge.net/projects/arma/files/armadillo-4.650.2.tar.gz | tar -xz
WORKDIR armadillo-4.650.2/build
RUN cmake .. -GNinja \
             -DCMAKE_CXX_FLAGS='-fuse-linker-plugin -flto -fno-fat-lto-objects' \
             -DCMAKE_INSTALL_PREFIX=$TOOLCHAIN_PATH
RUN ninja
RUN ninja install
# Fix up our armadillo configuration
RUN sed -i 's/^\/\* #undef ARMA_USE_LAPACK \*\//#define ARMA_USE_LAPACK/' $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp \
 && sed -i 's/^#define ARMA_USE_WRAPPER/\/\/ #define ARMA_USE_WRAPPER/'   $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp \
 && sed -i 's/^\/\/ #define ARMA_USE_CXX11/#define ARMA_USE_CXX11/'       $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp \
 && sed -i 's/^\/\/ #define ARMA_USE_U64S64/#define ARMA_USE_U64S64/'     $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp
WORKDIR /tmp
RUN rm -rf armadillo-4.650.2

# Catch
RUN wget https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp -O "$TOOLCHAIN_PATH/include/catch.hpp"

# TCMalloc
WORKDIR /tmp
RUN curl -L https://googledrive.com/host/0B6NtGsLhIcf7MWxMMF9JdTN3UVk/gperftools-2.4.tar.gz | tar -xz
WORKDIR gperftools-2.4/build
RUN CFLAGS="-march=atom -mtune=atom -fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    CXXFLAGS="-march=atom -mtune=atom -fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    LDFLAGS="-L$TOOLCHAIN_PATH/lib" \
    ../configure \
    --with-tcmalloc-pagesize=64 \
    --enable-minimal \
    --prefix=$TOOLCHAIN_PATH
RUN make
RUN make install
WORKDIR /tmp
RUN rm -rf gperftools-2.4

# Boost (remove when yaml-cpp bumps to 0.6)
RUN apt-get -y install libboost-dev

# yaml-cpp
WORKDIR /tmp
RUN curl -L https://yaml-cpp.googlecode.com/files/yaml-cpp-0.5.1.tar.gz | tar -xz
WORKDIR yaml-cpp-0.5.1/build
RUN cmake .. -DCMAKE_CXX_FLAGS='-fuse-linker-plugin -flto -fno-fat-lto-objects' \
             -DYAML_CPP_BUILD_CONTRIB=OFF \
             -DYAML_CPP_BUILD_TOOLS=OFF \
             -DCMAKE_INSTALL_PREFIX=$TOOLCHAIN_PATH
RUN make
RUN make install
WORKDIR /tmp
RUN rm -rf yaml-cpp-0.5.1

# ncurses
WORKDIR /tmp
RUN curl -L http://ftp.gnu.org/pub/gnu/ncurses/ncurses-5.9.tar.gz | tar -xz
WORKDIR ncurses-5.9
RUN CFLAGS="-fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    CXXFLAGS="-fuse-linker-plugin -flto -fno-fat-lto-objects -I$TOOLCHAIN_PATH/include -O3" \
    LDFLAGS="-L$TOOLCHAIN_PATH/lib" \
    ./configure \
    --without-progs \
    --without-tests \
    --prefix=$TOOLCHAIN_PATH
RUN make
RUN make install
# # Build dependencies
# RUN apt-get -y install git-core
# RUN apt-get -y install build-essential
# RUN apt-get -y install cmake
# RUN apt-get -y install ninja-build
# RUN apt-get -y install bibtool
# RUN apt-get -y install libgoogle-perftools-dev
# RUN apt-get -y install libmatheval-dev
# RUN apt-get -y install libboost-dev

# # zeromq
# RUN add-apt-repository ppa:chris-lea/zeromq
# RUN apt-get -y install libzmq3-dev

# # Install and configure icecream
# RUN apt-get -y install icecc
# RUN sed -i -e '/ICECC_SCHEDULER_HOST=/ s/="[a-zA-Z0-9]+/="10.1.0.80"/' /etc/icecc/icecc.conf
# ENV PATH /usr/lib/icecc/bin:$PATH

# # Download and install cppformat
# WORKDIR /tmp
# RUN git clone https://github.com/cppformat/cppformat
# WORKDIR /tmp/cppformat/build
# RUN cmake .. -GNinja
# RUN ninja
# RUN ninja install

# # NUClear dependencies
# RUN apt-get -y install libprotobuf-dev
# RUN apt-get -y install libespeak-dev
# RUN apt-get -y install librtaudio-dev
# RUN apt-get -y install libncurses5-dev
# RUN apt-get -y install libjpeg-turbo8-dev
# RUN apt-get -y install libfftw3-dev
# RUN apt-get -y install libaubio-dev
# RUN apt-get -y install libsndfile-dev
# RUN apt-get -y install libyaml-cpp-dev
# RUN apt-get -y install protobuf-compiler

# # NUClear
# WORKDIR /tmp
# RUN git clone -b OldDSL --single-branch https://github.com/fastcode/nuclear NUClear
# WORKDIR /tmp/NUClear/build
# RUN cmake .. -GNinja -DNUCLEAR_BUILD_TESTS=OFF
# RUN ninja
# RUN ninja install

# RUN apt-get -y install libopenblas-dev
# RUN apt-get -y install liblapack-dev
# RUN apt-get -y install libffi-dev
# RUN add-apt-repository ppa:comp-phys/stable
# RUN apt-get -y install libarmadillo-dev
# RUN apt-get -y install python
# RUN apt-get -y install wget

# # Catch
# WORKDIR /usr/local/include/
# RUN wget https://raw.github.com/philsquared/Catch/5ecb72b9bb65cd8fed2aec4da23a3bc21bbccd74/single_include/catch.hpp

# # Quex
# WORKDIR /tmp
# RUN wget https://downloads.sourceforge.net/project/quex/DOWNLOAD/quex-0.65.2.tar.gz -O quex-0.65.2.tar.gz
# RUN tar -zxf quex-0.65.2.tar.gz
# RUN mv quex-0.65.2/ /usr/local/etc/quex
# RUN ln -s /usr/local/etc/quex/quex/ /usr/local/include/quex
# RUN echo '#!/bin/bash' > /usr/local/bin/quex
# RUN echo 'QUEX_PATH=/usr/local/etc/quex python /usr/local/etc/quex/quex-exe.py "$@"' >> /usr/local/bin/quex
# RUN chmod +x /usr/local/bin/quex

# # Useful Tools
# RUN apt-get -y install vim
# RUN apt-get -y install cmake-curses-gui

# You need to mount your local code directory to the container
# Eg: docker run -t -i -v /local/path/:/nubots/NUbots /bin/bash
VOLUME /nubots/NUbots
WORKDIR /nubots/NUbots/build

# Expose our NUbugger ports to allow running in the container
EXPOSE 12000 12001
