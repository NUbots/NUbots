# == Class: nuclearport
#
# Clones and builds NUClearPort in the specified directory as the specified user
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
        Package['libboost-math-dev'],
        Package['libarmadillo-dev'],
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

