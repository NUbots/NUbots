
class nuclearport_dev_dependencies (
    $username = 'nubot',
  ) {
  class { 'vim': 
    username => $username,
  }

  package { 'build-essential': ensure => latest }
  package { 'cmake': ensure => latest }
  package { 'git': ensure => latest }
  package { 'openssh-server': ensure => latest }
  package { 'libzmq-dev': ensure => latest }
  package { ['libprotobuf-dev', 'protobuf-compiler']: ensure => latest }
  package { 'libespeak-dev': ensure => latest }
  package { 'librtaudio-dev': ensure => latest }
  package { 'libncurses5-dev': ensure => latest }
  package { 'libjpeg-turbo8-dev': ensure => latest }
  package { 'libfftw3-dev': ensure => latest }
  package { 'libaubio-dev': ensure => latest }
  package { 'libsndfile-dev': ensure => latest }
}

node default {
  # class { 'nuclearport_dev_dependencies': }
}

node npvagrant {
  # Perform a single `apt-get update` before installing ANY packages.
  exec { "initial_apt_update":
    command => "/usr/bin/apt-get update"
  } -> Package <| |>

  $username = 'vagrant'
  $nubots_dir = "/home/${username}/nubots"

  class { 'nuclearport_dev_dependencies':
    username => $username,
  }
  file { 'nubots_dir':
    path => "/home/${username}/nubots",
    ensure => directory,
    owner => $username,
    group => $username,
  }
  class { 'nuclear':
    username => $username,
    nubots_dir => $nubots_dir,
  }
}
