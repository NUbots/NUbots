
class nubots_nuclearport_dev_vm (
    $username = 'nubot',
  ) {
  class { 'vim':
    username => $username,
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

  file { 'nubots_dir':
    path => "/home/${username}/NUbots",
    ensure => directory,
    owner => $username,
    group => $username,
  }

  class { 'nuclearPort':
    username => $username,
  }
}

node default {
  class { 'nubots_nuclearport_dev_vm': }
}

node mitchell-VirtualBox {
  class { 'nubots_nuclearport_dev_vm':
    username => 'mitchell',
  }
}
