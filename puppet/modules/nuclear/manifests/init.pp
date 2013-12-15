# == Class: nuclear
#
# Installs the NUClear message passing framework development files.
#
class nuclear(
    $username = 'nubot',
    $nubots_dir = "/home/${username}/NUbots", #"
    $clone_directory = "/home/${username}/NUbots/NUClear", #"
  ) {
  include gcc48
  include catch
  include zmq

  $nuclear_build_dir = "${nubots_dir}/NUClear/build"

  vcsrepo { 'nuclear_repo':
    require => [File['nubots_dir'], Package['git']],
    path => "${nubots_dir}/NUClear",
    source => "https://github.com/Fastcode/NUClear.git",
    provider => 'git',
    ensure => present,
    user => $username,
    owner => $username,
  } ->
  file { 'nuclear_build_dir':
    path => $nuclear_build_dir,
    ensure => directory,
    purge => true,
    force => true,
    owner => $username,
    group => $username,
  } ~>
  exec { 'nuclear_cmake':
    require => [
        Class['gcc48'],
        Class['catch'],
        Class['zmq'],
        Package['build-essential'],
        Package['cmake'],
        Package['protobuf-compiler'],
      ],
    command => 'cmake ..',
    cwd => $nuclear_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
    user => $username,
  } ~>
  exec { 'nuclear_make':
    command => 'make -j',
    cwd => $nuclear_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
    user => $username,
  } ~>
  exec { 'nuclear_install':
    command => 'make install',
    cwd => $nuclear_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
  }
}

