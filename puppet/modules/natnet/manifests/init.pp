# == Class: NatNet
#
# Installs NatNet
#
class natnet(
    $username = 'nubot',
    $dir = "/home/${username}/NatNetLinux", #"
  ) {

  $build_dir = "$dir/build"

  package { 'libboost-dev': ensure => latest, }
  package { 'libboost-thread-dev': ensure => latest, }
  package { 'libboost-system-dev': ensure => latest, }

  file { 'natnet_dir':
    path => $dir,
    ensure => directory,
    purge => true,
    force => true,
    owner => $username,
    group => $username,
  } ~>
  vcsrepo { 'natnet_repo':
    require => [File['natnet_dir'], Package['git']],
    path => $dir,
    source => "https://github.com/rocketman768/NatNetLinux.git",
    provider => 'git',
    ensure => present,
    user => $username,
    owner => $username,
  } ->
  file { 'natnet_build_dir':
    path => $build_dir,
    ensure => directory,
    purge => true,
    force => true,
    owner => $username,
    group => $username,
  } ~>
  exec { 'natnet_cmake':
    command => 'cmake .. -DBUILD_EXAMPLES=OFF',
    cwd => $build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
    user => $username
  } ~>
  exec { 'natnet_make':
    command => 'make -j',
    cwd => $build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
    user => $username
  } ~>
  exec { 'natnet_install':
    command => 'make install',
    cwd => $build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
  }
}

