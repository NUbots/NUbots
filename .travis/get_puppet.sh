#!/bin/bash

# Setup ruby so puppet works
rvm install ruby --latest

# Download the puppetlabs deb file to install their apt repo
sudo wget -N http://apt.puppetlabs.com/puppetlabs-release-trusty.deb -O $TRAVIS_BUILD_DIR/toolchain/puppetlabs-release-trusty.deb
sudo dpkg -i $TRAVIS_BUILD_DIR/toolchain/puppetlabs-release-trusty.deb

# Update, and then install puppet
sudo apt-get update
sudo apt-get install puppet

# Install the puppet modules that are required for this install
sudo mkdir -p /etc/puppet/modules;
sudo puppet module install puppetlabs-apt --module_repository https://forge.puppet.com --version 2.4.0
sudo puppet module install puppetlabs-vcsrepo --module_repository https://forge.puppet.com
sudo puppet module install camptocamp-archive --module_repository https://forge.puppet.com
sudo puppet module install maestrodev-wget --module_repository https://forge.puppet.com
