# == Class: robocup::build_dep
#
# Installs all build-dependencies for NUClearPort
#
class robocup::build_dep (
    $username = 'nubot',
    $nubots_dir = "/home/${username}/nubots"
  ) {
  package { 'build-essential': ensure => latest }
  package { 'cmake': ensure => latest }
  include ninja
  package { 'git': ensure => latest }
  # package { 'openssh-server': ensure => latest }
  package { ['libprotobuf-dev', 'protobuf-compiler']: ensure => latest }
  package { 'libespeak-dev': ensure => latest }
  package { 'librtaudio-dev': ensure => latest }
  package { 'libncurses5-dev': ensure => latest }
  package { 'libjpeg-turbo8-dev': ensure => latest }
  package { 'libfftw3-dev': ensure => latest }
  package { 'libaubio-dev': ensure => latest }
  package { 'libsndfile-dev': ensure => latest } # virtual package
  # package { 'libsndfile1-dev': ensure => latest } # we only need this?
  package { ['libboost-dev', 'libyaml-cpp-dev']: ensure => latest }
  package { 'libmuparser-dev': ensure => latest }
  package { 'libopenblas-dev': ensure => latest }
  # package { 'liblapack-dev': ensure => latest }
  package { 'libtcmalloc-minimal4': ensure => latest }
  package { 'libgoogle-perftools-dev': ensure => latest }

  include armadillo
  include quex

  class { 'nuclear':
    username => $username,
    nubots_dir => $nubots_dir,
  }

  file { 'nubots_dir':
    path => $nubots_dir,
    ensure => directory,
    owner => $username,
    group => $username,
  }
}
