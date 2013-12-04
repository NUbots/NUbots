# == Class: zmq
#
# Installs the development files for ZeroMQ version 3
#
class zmq {
  include apt

  apt::ppa { 'ppa:chris-lea/zeromq': }

  package { 'libzmq3-dev':
    ensure => latest,
    require => Apt::Ppa['ppa:chris-lea/zeromq'],
  }
}
