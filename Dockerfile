FROM 32bit/ubuntu:14.04
MAINTAINER Simon Hartcher "simon@simonhartcher.com"
MAINTAINER Trent Houliston "trent@houliston.me"
ENV HOSTNAME nubotsvm
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive

# Why do you still need keys for the Ubuntu extras repository
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 437D05B5 3E5C1192

RUN dpkg-divert --local --rename --add /sbin/initctl
RUN ln -s /bin/true /sbin/initctl

RUN apt-get update

# Need this for apt-add-repository
RUN apt-get -y install software-properties-common

# We use g++4.9
RUN add-apt-repository ppa:ubuntu-toolchain-r/test

# Build dependencies
RUN apt-get -y install git-core
RUN apt-get -y install build-essential
RUN apt-get -y install cmake
RUN apt-get -y install ninja-build
RUN apt-get -y install bibtool
RUN apt-get -y install libgoogle-perftools-dev
RUN apt-get -y install libmatheval-dev
RUN apt-get -y install libboost-dev

# zeromq
RUN add-apt-repository ppa:chris-lea/zeromq
RUN apt-get -y install libzmq3-dev

# Install and configure icecream
RUN apt-get -y install icecc
RUN sed -i -e '/ICECC_SCHEDULER_HOST=/ s/="[a-zA-Z0-9]+/="10.1.0.80"/' /etc/icecc/icecc.conf
ENV PATH /usr/lib/icecc/bin:$PATH

# Download and install cppformat
WORKDIR /tmp
RUN git clone https://github.com/cppformat/cppformat
WORKDIR /tmp/cppformat/build
RUN cmake .. -GNinja
RUN ninja
RUN ninja install

# NUClear dependencies
RUN apt-get -y install libprotobuf-dev
RUN apt-get -y install libespeak-dev
RUN apt-get -y install librtaudio-dev
RUN apt-get -y install libncurses5-dev
RUN apt-get -y install libjpeg-turbo8-dev
RUN apt-get -y install libfftw3-dev
RUN apt-get -y install libaubio-dev
RUN apt-get -y install libsndfile-dev
RUN apt-get -y install libyaml-cpp-dev
RUN apt-get -y install protobuf-compiler

# NUClear
WORKDIR /tmp
RUN git clone -b OldDSL --single-branch https://github.com/fastcode/nuclear NUClear
WORKDIR /tmp/NUClear/build
RUN cmake .. -GNinja -DNUCLEAR_BUILD_TESTS=OFF
RUN ninja
RUN ninja install

RUN apt-get -y install libopenblas-dev
RUN apt-get -y install liblapack-dev
RUN apt-get -y install libffi-dev
RUN add-apt-repository ppa:comp-phys/stable
RUN apt-get -y install libarmadillo-dev
RUN apt-get -y install python
RUN apt-get -y install wget

# Catch
WORKDIR /usr/local/include/
RUN wget https://raw.github.com/philsquared/Catch/5ecb72b9bb65cd8fed2aec4da23a3bc21bbccd74/single_include/catch.hpp

# Quex
WORKDIR /tmp
RUN wget https://downloads.sourceforge.net/project/quex/DOWNLOAD/quex-0.65.2.tar.gz -O quex-0.65.2.tar.gz
RUN tar -zxf quex-0.65.2.tar.gz
RUN mv quex-0.65.2/ /usr/local/etc/quex
RUN ln -s /usr/local/etc/quex/quex/ /usr/local/include/quex
RUN echo '#!/bin/bash' > /usr/local/bin/quex
RUN echo 'QUEX_PATH=/usr/local/etc/quex python /usr/local/etc/quex/quex-exe.py "$@"' >> /usr/local/bin/quex
RUN chmod +x /usr/local/bin/quex

# Useful Tools
RUN apt-get -y install vim
RUN apt-get -y install cmake-curses-gui

# You need to mount your local code directory to the container
# Eg: docker run -t -i -v /local/path/:/nubots/NUbots /bin/bash
VOLUME /nubots/NUbots
WORKDIR /nubots/NUbots/build

# Expose our NUbugger ports to allow running in the container
EXPOSE 12000 12001
