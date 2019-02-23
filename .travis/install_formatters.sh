#!/bin/bash

sudo apt-add-repository -y ppa:ubuntu-toolchain-r/test

# Add the llvm 6.0 repo to our sources and add its key
wget -O - http://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add -
sudo apt-add-repository -y 'deb http://apt.llvm.org/trusty/ llvm-toolchain-trusty-6.0 main'

# Update and install clang-format
sudo apt-get update
sudo apt-get install clang-format-6.0
sudo apt-get install colordiff
