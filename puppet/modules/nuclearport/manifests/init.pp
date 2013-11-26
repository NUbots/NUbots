# == Class: nuclearport
#
# Full description of class nuclearport here.
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
#  class { nuclearport:
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
class nuclearport(
    $username = 'nubot',
    $nubots_dir = "/home/${username}/NUbots", #"
  ) {
  include gcc48

  $nuclearport_build_dir = "${nubots_dir}/NUClearPort/build"

  vcsrepo { 'nuclearport_repo':
    require => [File["${nubots_dir}"], Package['git']],
    path => "${nubots_dir}/NUClearPort",
    source => "https://github.com/nubots/NUClearPort.git",
    provider => 'git',
    ensure => present,
    user => $username,
    owner => $username,
  } ->
  file { 'nuclearport_build_dir':
    path => $nuclearport_build_dir,
    ensure => directory,
    purge => true,
    force => true,
    owner => $username,
    group => $username,
  } ~>
  exec { 'nuclearport_cmake':
    require => [
        Class['gcc48'],
        Class['nuclear'],
        Package['build-essential'],
        Package['libespeak-dev'],
        Package['librtaudio-dev'],
        Package['libncurses5-dev'],
        Package['libjpeg-turbo8-dev'],
        Package['libfftw3-dev'],
        Package['libaubio-dev'],
      ],
    command => 'cmake ..',
    cwd => $nuclearport_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
    user => $username,
  } ~>
  exec { 'nuclearport_make':
    command => 'make -j',
    cwd => $nuclearport_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
    user => $username,
  }
}

