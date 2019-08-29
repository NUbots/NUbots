#!/bin/bash

# Download and install our current toolchain version
cd $DOCKER_BUILD_DIR/toolchain
sudo wget -N http://nubots.net/debs/nubots-toolchain-3.0.4.deb
sudo dpkg -i nubots-toolchain-3.0.4.deb

# Setup ruby so puppet works
rvm install ruby --latest

# Update, and then install puppet
sudo apt-get update
sudo apt-get install -y --reinstall puppet

# Install the puppet modules that are required for this install
sudo mkdir -p /etc/puppet/modules;
sudo puppet module install puppetlabs-apt --module_repository https://forge.puppet.com --version 2.4.0
sudo puppet module install puppetlabs-vcsrepo --module_repository https://forge.puppet.com
sudo puppet module install camptocamp-archive --module_repository https://forge.puppet.com
sudo puppet module install maestrodev-wget --module_repository https://forge.puppet.com

# Change back to our build dir
cd $DOCKER_BUILD_DIR

# Apply the puppet file to the vm
sudo puppet apply --parser=future --verbose --debug --modulepath=puppet/modules:/etc/puppet/modules puppet/manifests/travis.pp
