#!/bin/bash

# Add the llvm 4.0 repo to our sources and add its key
wget -O - http://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add -
sudo apt-add-repository -y 'deb http://apt.llvm.org/trusty/ llvm-toolchain-trusty-5.0 main'

# Update and install clang-format
sudo apt-get update
sudo apt-get install clang-format-5.0
sudo apt-get install colordiff
