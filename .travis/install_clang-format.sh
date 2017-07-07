#!/bin/bash

# Add the llvm 4.0 repo to our sources and add its key
sudo echo "deb http://apt.llvm.org/trusty/ llvm-toolchain-trusty-4.0 main" | sudo tee /etc/apt/sources.list.d/llvm.list
wget -O - http://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add -

# Update and install clang-format
sudo apt-get update
sudo apt-get install clang-format
