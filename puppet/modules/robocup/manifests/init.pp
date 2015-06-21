# == Class: robocup
#
# Clones and builds NUClearPort in the specified directory as the specified user
#
class robocup (
    $username = 'nubot',
    $nubots_dir = "/home/${username}/NUbots"
  ) {
  $robocup_build_dir = "${nubots_dir}/NUClearPort/build"

  vcsrepo { 'robocup_repo':
    require => [File["${nubots_dir}"], Package['git']],
    path => "${nubots_dir}/NUbots",
    source => "https://github.com/NUbots/NUbots.git",
    provider => 'git',
    ensure => present,
    user => $username,
    owner => $username,
  } ->
  file { 'robocup_build_dir':
    path => $robocup_build_dir,
    ensure => directory,
    purge => true,
    force => true,
    owner => $username,
    group => $username,
  } ~>
  exec { 'robocup_cmake':
    require => [
        Class['robocup::build_dep'],
      ],
    command => 'cmake ..',
    cwd => $robocup_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
    user => $username,
  } ~>
  exec { 'robocup_make':
    command => 'make -j',
    cwd => $robocup_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
    user => $username,
  }
}

