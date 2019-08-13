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
COPY --chown=nubots:nubots toolchain/${platform}.cmake /usr/local/toolchain.cmake
COPY --chown=nubots:nubots toolchain/${platform}.sh /usr/local/toolchain.sh

# Copy over general install tools
COPY --chown=nubots:nubots package/install-autotools.sh /usr/local/bin/install-autotools.sh
COPY --chown=nubots:nubots package/install-cmake.sh /usr/local/bin/install-cmake.sh

# Copy across the individual libraries needed and build them
COPY --chown=nubots:nubots package/zlib.sh /usr/local/package/zlib.sh
RUN /usr/local/package/zlib.sh

COPY --chown=nubots:nubots package/protobuf.sh /usr/local/package/protobuf.sh
RUN /usr/local/package/protobuf.sh
