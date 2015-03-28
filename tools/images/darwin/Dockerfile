FROM 32bit/ubuntu:14.04
MAINTAINER Simon Hartcher "simon@simonhartcher.com"
MAINTAINER Trent Houliston "trent@houliston.me"
ENV HOSTNAME nubotsvm
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive

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
                       nano \
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

# Get NUClear dependencies
RUN apt-get -y install libprotobuf-dev \
                       protobuf-compiler \
                       libzmq3-dev

# Build and install NUClear
WORKDIR /tmp
RUN git clone -b OldDSL --depth 1 --single-branch https://github.com/FastCode/NUClear NUClear \
 && cd NUClear \
 && cmake -GNinja -DNUCLEAR_BUILD_TESTS=OFF \
 && ninja \
 && ninja install \
 && cd .. \
 && rm -rf NUClear

# Build dependencies
RUN apt-get -y install libgoogle-perftools-dev
RUN apt-get -y install libmuparser-dev
RUN apt-get -y install libboost-dev
RUN apt-get -y install libespeak-dev
RUN apt-get -y install librtaudio-dev
RUN apt-get -y install libncurses5-dev
RUN apt-get -y install libjpeg-turbo8-dev
RUN apt-get -y install libfftw3-dev
RUN apt-get -y install libaubio-dev
RUN apt-get -y install libsndfile-dev
RUN apt-get -y install libyaml-cpp-dev
RUN apt-get -y install libopenblas-dev
RUN apt-get -y install liblapack-dev

# Install and configure icecream
RUN apt-get -y install icecc \
 && sed -i 's/ICECC_SCHEDULER_HOST=""/ICECC_SCHEDULER_HOST="10.1.0.80"/' /etc/icecc/icecc.conf
ENV PATH /usr/lib/icecc/bin:$PATH

# Download and install cppformat
RUN git clone --depth 1 --single-branch https://github.com/cppformat/cppformat \
 && cd cppformat \
 && cmake -DCMAKE_CXX_FLAGS="-O3" \
          -DCMAKE_C_FLAGS="-O3" \
 && make -j$(nproc) \
 && make install \
 && cd .. \
 && rm -rf cppformat

#Armadillo
WORKDIR /tmp
RUN wget http://sourceforge.net/projects/arma/files/armadillo-4.650.3.tar.gz
RUN tar -zxf armadillo-4.650.3.tar.gz
WORKDIR /tmp/armadillo-4.650.3
RUN cmake .
RUN make install
RUN sed -i 's/^\/\* #undef ARMA_USE_LAPACK \*\//#define ARMA_USE_LAPACK/' /usr/include/armadillo_bits/config.hpp
RUN sed -i 's/^#define ARMA_USE_WRAPPER/\/\/ #define ARMA_USE_WRAPPER/'   /usr/include/armadillo_bits/config.hpp
RUN sed -i 's/^\/\/ #define ARMA_USE_CXX11/#define ARMA_USE_CXX11/'       /usr/include/armadillo_bits/config.hpp
RUN sed -i 's/^\/\/ #define ARMA_USE_U64S64/#define ARMA_USE_U64S64/'     /usr/include/armadillo_bits/config.hpp

# Catch
WORKDIR /usr/local/include/
RUN wget https://raw.github.com/philsquared/Catch/5ecb72b9bb65cd8fed2aec4da23a3bc21bbccd74/single_include/catch.hpp

# Quex
WORKDIR /tmp
RUN wget https://downloads.sourceforge.net/project/quex/DOWNLOAD/quex-0.65.4.tar.gz -O quex-0.65.4.tar.gz
RUN tar -zxf quex-0.65.4.tar.gz
RUN mv quex-0.65.4/ /usr/local/etc/quex
RUN ln -s /usr/local/etc/quex/quex/ /usr/local/include/quex
RUN echo '#!/bin/bash' > /usr/local/bin/quex
RUN echo 'QUEX_PATH=/usr/local/etc/quex python /usr/local/etc/quex/quex-exe.py "$@"' >> /usr/local/bin/quex
RUN chmod +x /usr/local/bin/quex

# You need to mount your local code directory to the container
# Eg: docker run -t -i -v /local/path/:/nubots/NUbots /bin/bash
VOLUME /nubots/NUbots
WORKDIR /nubots/NUbots/build

# Expose our NUbugger ports to allow running in the container
EXPOSE 12000 12001
