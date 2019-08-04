FROM alpine:edge

# Protobuf is installed in 3 different places. 
# Once for the sytem, once for python, and once for the toolchain.
# We need to maintain the same version across all 3 installations.
ARG PROTOBUF_VERSION="3.8.0"

# Install user and build tools
# We need the testing repository for some packages
RUN echo "http://dl-cdn.alpinelinux.org/alpine/edge/testing" >> /etc/apk/repositories \
    && apk update \
    && apk add --no-cache gcc g++ gfortran binutils-dev musl-dev linux-headers clang gdb valgrind dos2unix graphviz unzip \
    automake autoconf libtool intltool gtk-doc texinfo bison pcre-dev pkgconf rsync wget curl \
    ncurses-dev git libstdc++ ninja nasm libusb-dev gettext python3-dev zlib-dev libjpeg-turbo-dev \
    gcc-arm-none-eabi newlib-arm-none-eabi zsh vim nano openssh-server cmake cmake-bash-completion \
    protobuf-dev=${PROTOBUF_VERSION}-r0

# Install python packages
RUN pip3.7 install pyparsing pydotplus pygments stringcase tensorflow-gpu termcolor protobuf==${PROTOBUF_VERSION}.post1 pillow xxhash

ADD "files/find_robot_hosts.sh" "/usr/bin/find_robot_hosts.sh"
RUN chmod 755 "/usr/bin/find_robot_hosts.sh"
ADD "files/toolchain_init.sh" "/etc/profile/toolchain_init.sh"
RUN chmod 644 "/etc/profile/toolchain_init.sh"
ADD "files/hosts" "/etc/hosts"
RUN chmod 644 "/etc/hosts"

# Set up user
RUN useradd -ms /bin/bash nubots
USER nubots
WORKDIR /home/nubots

ADD --chown="nubots:nubots" "files/id_rsa" "/home/nubots/.ssh/id_rsa"
RUN chmod 600 "/home/nubots/.ssh/id_rsa"
ADD --chown="nubots:nubots" "files/id_rsa.pub" "/home/nubots/.ssh/id_rsa.pub"
RUN chmod 600 "/home/nubots/.ssh/id_rsa.pub"
ADD --chown="nubots:nubots" "files/ssh_config" "/home/nubots/.ssh/ssh_config"
