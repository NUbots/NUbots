# == Class: nuclear
#
# Full description of class nuclear here.
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
#  class { nuclear:
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
class nuclear {
  include gcc48
  include catch

  $nuclear_build_dir = '/home/mitchell/NUbots/NUClear/build'

  vcsrepo { 'nuclear_repo':
    require => [File['nubots_dir'], Package['git']],
    path => '/home/mitchell/NUbots/NUClear',
    source => "https://github.com/Fastcode/NUClear.git",
    provider => 'git',
    ensure => present,
    user => 'mitchell',
    owner => 'mitchell',
  } ->
  file { 'nuclear_build_dir':
    path => $nuclear_build_dir,
    ensure => directory,
    purge => true,
    force => true,
    owner => 'mitchell',
    group => 'mitchell',
  } ~>
  exec { 'nuclear_cmake':
    require => [
        Class['gcc48'],
        Class['catch'],
        Package['cmake'],
        Package['libzmq-dev'],
        Package['protobuf-compiler'],
      ],
    command => 'cmake ..',
    cwd => $nuclear_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
    user => 'mitchell',
  } ~>
  exec { 'nuclear_make':
    command => 'make -j',
    cwd => $nuclear_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
    user => 'mitchell',
  } ~>
  exec { 'nuclear_install':
    command => 'make install',
    cwd => $nuclear_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
  }
}

