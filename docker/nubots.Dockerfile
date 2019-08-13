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

FROM ubuntu:18.04
RUN apt-get update && apt-get -y install build-essential wget
RUN groupadd -r nubots && useradd --no-log-init -r -g nubots nubots

# Create the home directory owned by nubots
RUN mkdir -p /home/nubots && chown -R nubots:nubots /home/nubots

# Setup /usr/local owned by nubots and swap to the nubots user
RUN chown -R nubots:nubots /usr/local
USER nubots
WORKDIR /home/nubots

# Add ssh key for sshing into the robot
COPY --chown="nubots:nubots" "files/id_rsa" "/home/nubots/.ssh/id_rsa"
COPY --chown="nubots:nubots" "files/id_rsa.pub" "/home/nubots/.ssh/id_rsa.pub"
COPY --chown="nubots:nubots" "files/ssh_config" "/home/nubots/.ssh/ssh_config"

################################################
#  _____           _      _           _        #
# |_   _|__   ___ | | ___| |__   __ _(_)_ __   #
#   | |/ _ \ / _ \| |/ __| '_ \ / _` | | '_ \  #
#   | | (_) | (_) | | (__| | | | (_| | | | | | #
#   |_|\___/ \___/|_|\___|_| |_|\__,_|_|_| |_| #
################################################
ARG platform=generic

# Copy across the specific toolchain files for this image
COPY --chown=nubots:nubots toolchain/${platform}.sh /usr/local/toolchain.sh

# Copy over a tool to install simple standard conforming libraries from source
COPY --chown=nubots:nubots package/install-from-source /usr/local/bin/install-from-source
RUN ln -s /usr/local/bin/install-from-source /usr/local/bin/install-cmake-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-autotools-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-make-from-source \
    && ln -s /usr/local/bin/install-from-source /usr/local/bin/install-bjam-from-source

# Install tools and libraries from source
RUN install-from-source https://www.zlib.net/zlib-1.2.11.tar.gz
RUN install-from-source https://github.com/google/protobuf/releases/download/v3.9.1/protobuf-cpp-3.9.1.tar.gz \
    --with-zlib
