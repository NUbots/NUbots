#!/bin/bash

# Download and install our current toolchain version
cd $TRAVIS_BUILD_DIR/toolchain
sudo wget -N http://nubots.net/debs/nubots-toolchain-3.0.1-travis.deb
sudo dpkg -i nubots-toolchain-3.0.1-travis.deb

# Setup ruby so puppet works
rvm install ruby --latest

# Download the puppetlabs deb file to install their apt repo
sudo wget -N http://apt.puppetlabs.com/puppetlabs-release-trusty.deb
sudo dpkg -i puppetlabs-release-trusty.deb

# Update, and then install puppet
sudo apt-get update
sudo apt-get install puppet

# Install the puppet modules that are required for this install
sudo mkdir -p /etc/puppet/modules;
sudo puppet module install puppetlabs-apt --module_repository https://forge.puppet.com --version 2.4.0
sudo puppet module install puppetlabs-vcsrepo --module_repository https://forge.puppet.com
sudo puppet module install camptocamp-archive --module_repository https://forge.puppet.com
sudo puppet module install maestrodev-wget --module_repository https://forge.puppet.com

# Change back to our build dir
cd $TRAVIS_BUILD_DIR

# Apply the puppet file to the vm
sudo puppet apply --parser=future --verbose --debug --modulepath=puppet/modules:/etc/puppet/modules puppet/manifests/travis.pp

# For some reason it looks like we have to run update-alternative again on travis
sudo update-alternatives --remove-all gcc || true
sudo update-alternatives --remove-all g++ || true
sudo update-alternatives --remove-all gfortan || true
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 100 --slave /usr/bin/g++ g++ /usr/bin/g++-7 --slave /usr/bin/gfortran gfortran /usr/bin/gfortran-7
