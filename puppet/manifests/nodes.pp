class initial_apt_update {
  # Perform a single `apt-get update` before installing ANY packages.
  exec { "initial_apt_update":
    command => "/usr/bin/apt-get update"
  } -> Package <| |>
}

node nuclearportvm {
  include initial_apt_update

  # define variables for this node
  $username = 'vagrant'

  class { 'nuclearport::build_dep': username => $username, }

  class { 'vim':  username => $username, }
}

node nubuggervm {
  include initial_apt_update

  # define variables for this node
  $username = 'vagrant'

  # these (and most packages) should be virtual resources
  package { 'build-essential': ensure => latest }
  package { 'git': ensure => latest }

  file { 'nubots_dir':
    path => "/home/${username}/nubots",
    ensure => directory,
    owner => $username,
    group => $username,
  }

  class { 'nubugger': username => $username, }

  class { 'vim': username => $username, }
}

node packer-virtualbox-iso, packer-vmware-iso {
  include initial_apt_update

  $username = 'vagrant'

  class { 'nuclearport::build_dep': username => $username, }

  # nubugger::build_dep
  package { 'pkg-config': ensure => latest, }
  package { 'uuid-dev': ensure => latest, }

  class { 'ruby':
    gems_version  => 'latest'
  }
  class { 'nodejs':
    version => 'stable',
    make_install => false,
  }

  # Non-essential developer tools:
  class { 'vim':  username => $username, }
  package { 'screen': ensure => latest, }
  package { 'htop': ensure => latest, }
}