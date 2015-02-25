FROM deevus/puppetbase
MAINTAINER Simon Hartcher "simon@simonhartcher.com"
ENV HOSTNAME nubotsvm

RUN apt-get update 
RUN apt-get -y install git-core 
RUN apt-get -y install cmake 
RUN apt-get -y install ninja-build 
RUN apt-get -y install bibtool 
RUN apt-get -y install libgoogle-perftools-dev 
RUN apt-get -y install libmatheval-dev
RUN apt-get -y install libboost-dev

ADD ./puppet /tmp/puppet
WORKDIR /tmp/puppet
RUN puppet apply --modulepath modules/ manifests/nodes.pp

WORKDIR /tmp
RUN git clone https://github.com/cppformat/cppformat
WORKDIR /tmp/cppformat
RUN cmake . -GNinja 
RUN ninja
RUN cp libformat.a /usr/local/lib
RUN cp format.h /usr/local/include

WORKDIR /nubots
RUN git clone -b OldDSL --single-branch https://github.com/fastcode/nuclear NUClear
WORKDIR /nubots/NUClear/build
RUN cmake .. -GNinja -DNUCLEAR_BUILD_TESTS=OFF
RUN ninja 
RUN ninja install

RUN apt-get -y install icecc
RUN sed -i -e '/ICECC_SCHEDULER_HOST=/ s/="[a-zA-Z0-9]+/="10.1.0.80"/' /etc/icecc/icecc.conf
ENV PATH /usr/lib/icecc/bin:$PATH

ADD . /nubots/NUbots
WORKDIR /nubots/NUbots/build
RUN rm -rf ./*
RUN cmake .. -GNinja
RUN ninja

