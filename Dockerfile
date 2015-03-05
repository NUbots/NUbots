FROM 32bit/ubuntu:14.04
MAINTAINER Simon Hartcher "simon@simonhartcher.com"
MAINTAINER Trent Houliston "trent@houliston.me"
ENV HOSTNAME nubotsvm
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive

# Set our toolchain path
ENV TOOLCHAIN_PATH /nubots/toolchain
ENV CMAKE_PREFIX_PATH /nubots/toolchain

# Set the extra flags that will be used while compiling
ENV COMPILER_FLAGS -march=atom -mtune=atom -fuse-linker-plugin -flto -fno-fat-lto-objects

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
RUN apt-get -y install vim \
                       wget \
                       curl \
                       git \
                       python \
                       unzip

# Get our build tools
RUN apt-get -y install build-essential \
                       gfortran \
                       cmake \
                       cmake-curses-gui \
                       ninja-build

# Change our default linker to gold
RUN update-alternatives --install /usr/bin/ld ld /usr/bin/ld.bfd 10 \
 && update-alternatives --install /usr/bin/ld ld /usr/bin/ld.gold 20

# Fix our ar, ranlib and nm tools to use the gcc versions
RUN echo '#/bin/bash'                                                                    > /usr/local/bin/ar \
 && echo '/usr/bin/ar $@ --plugin /usr/lib/gcc/i686-linux-gnu/4.9/liblto_plugin.so'     >> /usr/local/bin/ar \
 && chmod +x /usr/local/bin/ar \
 && echo '#/bin/bash'                                                                    > /usr/local/bin/ranlib \
 && echo '/usr/bin/ranlib $@ --plugin /usr/lib/gcc/i686-linux-gnu/4.9/liblto_plugin.so' >> /usr/local/bin/ranlib \
 && chmod +x /usr/local/bin/ranlib \
 && echo '#/bin/bash'                                                                    > /usr/local/bin/nm \
 && echo '/usr/bin/nm $@ --plugin /usr/lib/gcc/i686-linux-gnu/4.9/liblto_plugin.so'     >> /usr/local/bin/nm \
 && chmod +x /usr/local/bin/nm

# build our libraries
WORKDIR /tmp

# Make our library installation script for autotools and cmake
RUN echo '#/bin/bash' >> autotools_install \
 && echo 'set -e' >> autotools_install \
 && echo 'curl -L $1 | tar xz' >> autotools_install \
 && echo 'shift 1' >> autotools_install \
 && echo 'BASE_DIR=$(pwd)' >> autotools_install \
 && echo 'CODE_DIR=$(ls -d */)' >> autotools_install \
 && echo 'cd "$CODE_DIR$CODE_PATH"' >> autotools_install \
 && echo 'CFLAGS="$COMPILER_FLAGS -I$TOOLCHAIN_PATH/include -O3" \\' >> autotools_install \
 && echo 'CXXFLAGS="$COMPILER_FLAGS -I$TOOLCHAIN_PATH/include -O3" \\' >> autotools_install \
 && echo 'LDFLAGS="-L$TOOLCHAIN_PATH/lib" \\' >> autotools_install \
 && echo './configure $@ \\' >> autotools_install \
 && echo '--prefix="$TOOLCHAIN_PATH"' >> autotools_install \
 && echo 'make -j$(nproc)' >> autotools_install \
 && echo 'make install' >> autotools_install \
 && echo 'rm -rf "$BASE_DIR/$CODE_DIR"' >> autotools_install \
 && chmod +x ./autotools_install

RUN echo '#/bin/bash' >> cmake_install \
 && echo 'set -e' >> cmake_install \
 && echo 'curl -L $1 | tar xz' >> cmake_install \
 && echo 'shift 1' >> cmake_install \
 && echo 'BASE_DIR=$(pwd)' >> cmake_install \
 && echo 'CODE_DIR=$(ls -d */)' >> cmake_install \
 && echo 'cd "$CODE_DIR$CODE_PATH"' >> cmake_install \
 && echo 'cmake \\' >> cmake_install \
 && echo '-DCMAKE_C_FLAGS="$COMPILER_FLAGS -O3" \\' >> cmake_install \
 && echo '-DCMAKE_CXX_FLAGS="$COMPILER_FLAGS -O3" \\' >> cmake_install \
 && echo '-DCMAKE_INSTALL_PREFIX="$TOOLCHAIN_PATH" \\' >> cmake_install \
 && echo '$@' >> cmake_install \
 && echo 'make -j$(nproc)' >> cmake_install \
 && echo 'make install' >> cmake_install \
 && echo 'rm -rf "$BASE_DIR/$CODE_DIR"' >> cmake_install \
 && chmod +x ./cmake_install

# zlib
RUN ./autotools_install http://zlib.net/zlib-1.2.8.tar.gz

# libprotobuf + protobuf-compiler
RUN ./autotools_install https://github.com/google/protobuf/releases/download/v2.6.1/protobuf-2.6.1.tar.gz --with-zlib

# openpgm
RUN CODE_PATH="openpgm/pgm" \
    ./autotools_install https://openpgm.googlecode.com/files/libpgm-5.2.122.tar.gz

# libzmq4
RUN OpenPGM_CFLAGS="" \
    OpenPGM_LIBS="" \
    ./autotools_install http://download.zeromq.org/zeromq-4.0.5.tar.gz \
 && wget https://raw.githubusercontent.com/zeromq/cppzmq/master/zmq.hpp -O "$TOOLCHAIN_PATH/include/zmq.hpp"

# NUClear
RUN git clone -b OldDSL --depth 1 --single-branch https://github.com/FastCode/NUClear NUClear \
 && cd NUClear \
 && cmake -GNinja \
          -DCMAKE_C_FLAGS="$COMPILER_FLAGS" \
          -DCMAKE_CXX_FLAGS="$COMPILER_FLAGS" \
          -DNUCLEAR_BUILD_TESTS=OFF \
          -DCMAKE_INSTALL_PREFIX="$TOOLCHAIN_PATH" \
 && ninja \
 && ninja install \
 && cd .. \
 && rm -rf NUClear

# OpenBLAS (includes lapack)
RUN curl -L https://github.com/xianyi/OpenBLAS/archive/v0.2.13.tar.gz | tar -xz \
 && cd OpenBLAS-0.2.13 \
 && TARGET=ATOM \
    USE_THREAD=1 \
    BINARY=32 \
    COMMON_OPT="$COMPILER_FLAGS -O3" \
    FCOMMON_OPT="$COMPILER_FLAGS -O3" \
    make \
 && make PREFIX="$TOOLCHAIN_PATH" install \
 && cd .. \
 && rm -rf OpenBLAS-0.2.13

# Armadillo
RUN ./cmake_install http://sourceforge.net/projects/arma/files/armadillo-4.650.2.tar.gz \
 && sed -i 's/^\/\* #undef ARMA_USE_LAPACK \*\//#define ARMA_USE_LAPACK/' $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp \
 && sed -i 's/^#define ARMA_USE_WRAPPER/\/\/ #define ARMA_USE_WRAPPER/'   $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp \
 && sed -i 's/^\/\/ #define ARMA_USE_CXX11/#define ARMA_USE_CXX11/'       $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp \
 && sed -i 's/^\/\/ #define ARMA_USE_U64S64/#define ARMA_USE_U64S64/'     $TOOLCHAIN_PATH/include/armadillo_bits/config.hpp

# Catch
RUN wget https://raw.githubusercontent.com/philsquared/Catch/master/single_include/catch.hpp -O "$TOOLCHAIN_PATH/include/catch.hpp"

# TCMalloc
RUN ./autotools_install https://googledrive.com/host/0B6NtGsLhIcf7MWxMMF9JdTN3UVk/gperftools-2.4.tar.gz \
    --with-tcmalloc-pagesize=64 \
    --enable-minimal

# Boost (remove when yaml-cpp bumps to 0.6)
RUN apt-get -y install libboost-dev

# yaml-cpp
RUN ./cmake_install https://yaml-cpp.googlecode.com/files/yaml-cpp-0.5.1.tar.gz \
    -DYAML_CPP_BUILD_CONTRIB=OFF \
    -DYAML_CPP_BUILD_TOOLS=OFF

# ncurses
RUN ./autotools_install http://ftp.gnu.org/pub/gnu/ncurses/ncurses-5.9.tar.gz \
    --without-progs \
    --without-tests

# fftw-3
RUN ./autotools_install http://www.fftw.org/fftw-3.3.4.tar.gz \
    --disable-fortran \
    --enable-shared

# Quex
RUN curl -L https://downloads.sourceforge.net/project/quex/DOWNLOAD/quex-0.65.2.tar.gz | tar -xz \
 && mkdir -p "$TOOLCHAIN_PATH/etc" \
 && mv quex-0.65.2 "$TOOLCHAIN_PATH/etc/quex" \
 && ln -s "$TOOLCHAIN_PATH/etc/quex/quex" "$TOOLCHAIN_PATH/include/quex" \
 && echo '#!/bin/bash' > "$TOOLCHAIN_PATH/bin/quex" \
 && echo "QUEX_PATH=$TOOLCHAIN_PATH/etc/quex python /usr/local/etc/quex/quex-exe.py \$@" >> "$TOOLCHAIN_PATH/bin/quex" \
 && chmod +x "$TOOLCHAIN_PATH/bin/quex"

# # cppformat
# # WORKDIR /tmp
# # RUN git clone --depth 1 --single-branch https://github.com/cppformat/cppformat
# # WORKDIR /tmp/cppformat/build
# # RUN cmake .. -GNinja \
# #              -DCMAKE_CXX_FLAGS="$COMPILER_FLAGS -O3" \
# #              -DCMAKE_C_FLAGS="$COMPILER_FLAGS -O3" \
# #              -DCMAKE_INSTALL_PREFIX="$TOOLCHAIN_PATH"
# # RUN ninja
# # RUN ninja install
# # WORKDIR /tmp
# # RUN rm -rf cppformat


# # jpeg-turbo
# # WORKDIR /tmp
# # RUN curl -L http://downloads.sourceforge.net/project/libjpeg-turbo/1.4.0/libjpeg-turbo-1.4.0.tar.gz | tar -xz


# # Alsa
# # WORKDIR /tmp
# # RUN curl -L ftp://ftp.alsa-project.org/pub/lib/alsa-lib-1.0.9.tar.bz2 | tar -xj
# # WORKDIR alsa-lib-1.0.9
# # RUN CFLAGS="$COMPILER_FLAGS -I$TOOLCHAIN_PATH/include -O3" \
# #     CXXFLAGS="$COMPILER_FLAGS -I$TOOLCHAIN_PATH/include -O3" \
# #     LDFLAGS="-L$TOOLCHAIN_PATH/lib" \
# #     ./configure \
# #     --prefix="$TOOLCHAIN_PATH"
# # RUN make
# # RUN make install
# # WORKDIR /tmp
# # RUN rm -rf alsa-lib-1.0.9

# # portaudio
# # WORKDIR /tmp
# # RUN curl -L http://www.portaudio.com/archives/pa_stable_v19_20140130.tgz | tar -xz
# # WORKDIR pa_stable_v19_20140130

# # espeak
# # WORKDIR /tmp
# # RUN wget http://sourceforge.net/projects/espeak/files/espeak/espeak-1.48/espeak-1.48.04-source.zip \
# #  && unzip espeak-1.48.04-source.zip \
# #  && rm espeak-1.48.04-source.zip
# # WORKDIR espeak-1.48.04-source/src

# # musllibc?
# # RUN curl -L http://www.musl-libc.org/releases/musl-1.1.6.tar.gz | tar -xz

# # libmatheval


# # RUN apt-get -y install libmatheval-dev

# # # Install and configure icecream
# # RUN apt-get -y install icecc
# # RUN sed -i -e '/ICECC_SCHEDULER_HOST=/ s/="[a-zA-Z0-9]+/="10.1.0.80"/' /etc/icecc/icecc.conf
# # ENV PATH /usr/lib/icecc/bin:$PATH

# # # Download and install cppformat
# # WORKDIR /tmp
# # RUN git clone https://github.com/cppformat/cppformat
# # WORKDIR /tmp/cppformat/build
# # RUN cmake .. -GNinja
# # RUN ninja
# # RUN ninja install

# # # NUClear dependencies
# # RUN apt-get -y install librtaudio-dev
# # RUN apt-get -y install libaubio-dev
# # RUN apt-get -y install libsndfile-dev

# # RUN apt-get -y install libffi-dev



# # You need to mount your local code directory to the container
# # Eg: docker run -t -i -v /local/path/:/nubots/NUbots /bin/bash
# VOLUME /nubots/NUbots
# WORKDIR /nubots/NUbots/build

# # Expose our NUbugger ports to allow running in the container
# EXPOSE 12000 12001
