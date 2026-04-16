#!/usr/bin/bash

bootstrap_sys(){
    install-package gcc-12-aarch64-linux-gnu g++-12-aarch64-linux-gnu binutils-aarch64-linux-gnu
    mkdir -p /l4t/targetfs
    wget -qO- http://cdimage.ubuntu.com/ubuntu-base/releases/jammy/release/ubuntu-base-22.04.5-base-arm64.tar.gz | tar -xz -C /l4t/targetfs;
}

setup_env(){
    # Restrict existing x86_64 repos to amd64 only before adding arm64 architecture
    sed -i 's/^deb \([^[]\)/deb [arch=amd64] \1/' /etc/apt/sources.list
    sed -i 's/^ *deb-src \([^[]\)/deb-src [arch=amd64] \1/' /etc/apt/sources.list

    dpkg --add-architecture arm64
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy main restricted universe multiverse" > /etc/apt/sources.list.d/ubuntu-ports.list
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list.d/ubuntu-ports.list
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-security main restricted universe multiverse" >> /etc/apt/sources.list.d/ubuntu-ports.list
    echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list.d/ubuntu-ports.list

    mkdir -p /l4t/targetfs/usr/local
    cd /l4t/targetfs/usr/local && ln -sf lib lib64
}

update_sys(){
    apt-get update
    apt-get upgrade -y
    apt-get autoremove -y
    rm -rf /var/cache/apt /var/lib/apt/lists/*
}

if [ "$1" != "generic" ]; then
    bootstrap_sys
    setup_env
fi
update_sys
