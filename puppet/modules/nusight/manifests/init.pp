# == Class: nusight
#
# Installs the NUsight debugging system.
#
class nusight(
    $username = 'nubot',
    $nubots_dir = "/home/${username}/nubots", #"
	) {
  # include nodejs
  include zmq

  $nusight_dir = "${nubots_dir}/NUsight"

  package { 'pkg-config': ensure => latest, }
  package { 'uuid-dev': ensure => latest, }
  package { 'nodejs': ensure => latest, }
  package { 'npm': ensure => latest, }

  vcsrepo { 'nusight_repo':
    require => [File['nubots_dir'], Package['git']],
    path => $nusight_dir,
    source => "https://github.com/nubots/NUsight.git",
    provider => 'git',
    ensure => present,
    user => $username,
    owner => $username,
    revision => 'develop',
  }

  exec { 'npm_install':
    require => [
        Package['libzmq3-dev'],
        Package['build-essential'],
        Package['pkg-config'],
        Package['uuid-dev'],
        Vcsrepo['nusight_repo'],
      ],
    command => 'npm install',
    cwd => $nusight_dir,
    path => $path,
    # refreshonly => true,
    logoutput => "on_failure",
    # user => $username,
    creates => "${nusight_dir}/node_modules"
  }
}
