# == Class: nubugger
#
# Installs the NUbugger debugging system.
#
class nubugger(
    $username = 'nubot',
    $nubots_dir = "/home/${username}/nubots", #"
	) {
  # include nodejs
  include zmq

  $nubugger_dir = "${nubots_dir}/NUbugger"

  package { 'pkg-config': ensure => latest, }
  package { 'uuid-dev': ensure => latest, }

  class { 'nodejs':
    version => 'stable',
    make_install => false,
  }

  vcsrepo { 'nubugger_repo':
    require => [File['nubots_dir'], Package['git']],
    path => $nubugger_dir,
    source => "https://github.com/nubots/NUbugger.git",
    provider => 'git',
    ensure => present,
    user => $username,
    owner => $username,
    revision => 'feature/puppet',
  } ~> 
  exec { 'npm_install':
    require => [
        Class['nodejs'],
        Class['zmq'],
        Package['build-essential'],
        Package['pkg-config'],
        Package['uuid-dev'],
        Vcsrepo['nubugger_repo'],
      ],
    command => 'npm install',
    cwd => $nubugger_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
    # user => $username,
  }
}
