#! /bin/sh

# Install OMPL dependencies:

# #   - Relevant boost libraries:
# sudo apt-get install libboost-system-dev
# sudo apt-get install libboost-filesystem-dev
# sudo apt-get install libboost-thread-dev
# sudo apt-get install libboost-serialization-dev
# sudo apt-get install libboost-program-options-dev
# sudo apt-get install libboost-test-dev
# sudo apt-get install libboost-chrono-dev
# sudo apt-get install libboost-date-time-dev

# #   - Eigen:
# sudo apt-get install libeigen3-dev

# Build and install OMPL:
mkdir -p /tmp/omplinstall
cd /tmp/omplinstall
git clone https://github.com/ompl/ompl.git
cd ompl
mkdir build
cd build
cmake ..
make
make install
cd /
rm -rf /tmp/omplinstall

# mkdir -p /tmp/omplinstall
# cd /tmp/omplinstall
# sudo apt-get install libffi-dev
# sudo apt-get install ruby-dev
# sudo gem install fpm
# git clone https://github.com/ompl/ompl.git
# cd ompl
# cd build
# mkdir build
# cmake .. -G Ninja
# ninja
# mkdir /tmp/omplinstall/debdir
# DESTDIR=/tmp/omplinstall/debdir ninja install
# fpm -s dir -t deb -n ompl -v 0.14.2 \
#                           -C /tmp/omplinstall/debdir \
#                           -d libboost-system-dev \
#                           -d libboost-filesystem-dev \
#                           -d libboost-thread-dev \
#                           -d libboost-serialization-dev \
#                           -d libboost-program-options-dev \
#                           -d libboost-test-dev \
#                           -d libboost-chrono-dev \
#                           -d libboost-date-time-dev \
#                           -d libeigen3-dev \
#                           usr/local/bin/ usr/local/include usr/local/lib usr/local/share
