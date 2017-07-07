#!/bin/bash

# Download and install our current toolchain version
cd $TRAVIS_BUILD_DIR/toolchain
sudo wget -N http://nubots.net/debs/nubots-toolchain-2.1.2-travis.deb
sudo dpkg -i nubots-toolchain-2.1.2-travis.deb
