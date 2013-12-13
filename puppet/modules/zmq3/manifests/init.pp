class zmq3 {
  include apt

  apt::ppa { 'ppa:chris-lea/zeromq': }

  package { 'libzmq3-dev':
    ensure => latest,
    require => Apt::Ppa['ppa:chris-lea/zeromq'], 
  }
}

