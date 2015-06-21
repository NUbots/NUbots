# == Class: ninja
class ninja {
  include apt

  apt::ppa { 'ppa:ubuntu-toolchain-r/test': }

  package { 'ninja-build':
    ensure => latest,
    require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'], 
  }
}
