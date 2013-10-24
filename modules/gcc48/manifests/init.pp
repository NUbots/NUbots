# == Class: gcc48
#
# Full description of class gcc48 here.
#
# === Parameters
#
# Document parameters here.
#
# [*sample_parameter*]
#   Explanation of what this parameter affects and what it defaults to.
#   e.g. "Specify one or more upstream ntp servers as an array."
#
# === Variables
#
# Here you should define a list of variables that this module would require.
#
# [*sample_variable*]
#   Explanation of how this variable affects the funtion of this class and if
#   it has a default. e.g. "The parameter enc_ntp_servers must be set by the
#   External Node Classifier as a comma separated list of hostnames." (Note,
#   global variables should be avoided in favor of class parameters as
#   of Puppet 2.6.)
#
# === Examples
#
#  class { gcc48:
#    servers => [ 'pool.ntp.org', 'ntp.local.company.com' ],
#  }
#
# === Authors
#
# Author Name <author@domain.com>
#
# === Copyright
#
# Copyright 2013 Your name here, unless otherwise noted.
#
class gcc48 {
  include apt

  apt::ppa { 'ppa:ubuntu-toolchain-r/test': }

  package { 'gcc-4.8':
    ensure => latest,
    require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'], 
  } ->
  exec { 'gcc-4.8_alternatives':
    command => 'update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.6',
    path => $path,
  }
  
  package { 'g++-4.8':
    ensure => latest,
    require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'], 
  } ->
  exec { 'g++-4.8_alternatives':
    command => 'update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 80 --slave /usr/bin/g++ g++ /usr/bin/g++-4.8',
    path => $path,
  }
}

