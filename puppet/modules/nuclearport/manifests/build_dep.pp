# nuclearport::build_dep class
#   Installs all build-dependencies for NUClearPort
class nuclearport::build_dep (
    $username = 'nubot',
    $nubots_dir => "/home/${username}/nubots",
  ) {
  package { 'build-essential': ensure => latest }
  package { 'cmake': ensure => latest }
  package { 'git': ensure => latest }
  package { 'openssh-server': ensure => latest }
  package { ['libprotobuf-dev', 'protobuf-compiler']: ensure => latest }
  package { 'libespeak-dev': ensure => latest }
  package { 'librtaudio-dev': ensure => latest }
  package { 'libncurses5-dev': ensure => latest }
  package { 'libjpeg-turbo8-dev': ensure => latest }
  package { 'libfftw3-dev': ensure => latest }
  package { 'libaubio-dev': ensure => latest }
  package { 'libsndfile-dev': ensure => latest }
  package { 'libboost-math-dev': ensure => latest }
  package { 'libarmadillo-dev': ensure => latest }

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
