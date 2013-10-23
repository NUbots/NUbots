
class nubots_nuclearport_dev_vm {
  file { 'nubots_dir':
    path => '/home/mitchell/NUbots',
    ensure => directory,
    owner => 'mitchell',
    group => 'mitchell',
  }

  package { 'cmake': ensure => latest }
  package { 'git': ensure => latest }
  package { 'openssh-server': ensure => latest }
  package { 'libzmq-dev': ensure => latest }
  package { ['libprotobuf-dev', 'protobuf-compiler']: ensure => latest }
  package { 'libespeak-dev': ensure => latest }
  package { 'librtaudio-dev': ensure => latest }
  package { 'libncurses5-dev': ensure => latest }
  
  package { 'libjpeg-dev': ensure => purged } ->
  package { 'libjpeg-turbo8-dev': ensure => latest }

  package { 'libfftw3-dev': ensure => latest }
  package { 'libaubio-dev': ensure => latest }
  include vim
  include nuclearPort
}

node default {
  include nubots_nuclearport_dev_vm
}

