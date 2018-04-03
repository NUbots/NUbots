class user_tools (String $user) {

  require build_tools

  # User tools
  package { 'vim': ensure => latest, }
  package { 'screen': ensure => latest, }
  package { 'clang-format-5.0': ensure => latest, }
  package { 'zsh': ensure => latest, }
  package { 'htop': ensure => latest, }
  package { 'gdb': ensure => latest, }
  package { 'cmake-curses-gui': ensure => latest, }
  package { 'dos2unix': ensure => latest, }
  package { 'unzip': ensure => latest, }
  package { 'graphviz': ensure => latest, }

  # INSTALL ROBOT HOST PARSER
  file { '/nubots/toolchain/find_robot_hosts.sh':
    ensure  => present,
    mode    => '755',
    source  => 'puppet:///modules/user_tools/find_robot_hosts.sh',
    replace => true,
  }

  # Set the users default shell to zsh
  user { "${user}": shell => '/bin/zsh', require => Package['zsh'], }

  # Setup prezto for a richer zsh experience
  vcsrepo { 'zprezto':
    ensure   => present,
    path     => "/home/${user}/.zprezto",
    provider => git,
    owner    => "${user}",
    group    => "${user}",
    source   => 'https://github.com/sorin-ionescu/prezto.git',
    require  => Package['git'],
  }

  # Create the required links for zprezto
  file { "/home/${user}/.zlogin":    ensure => link, target => "/home/${user}/.zprezto/runcoms/zlogin",    require => Vcsrepo['zprezto'], }
  file { "/home/${user}/.zlogout":   ensure => link, target => "/home/${user}/.zprezto/runcoms/zlogout",   require => Vcsrepo['zprezto'], }
  file { "/home/${user}/.zpreztorc": ensure => link, target => "/home/${user}/.zprezto/runcoms/zpreztorc", require => Vcsrepo['zprezto'], }
  file { "/home/${user}/.zprofile":  ensure => link, target => "/home/${user}/.zprezto/runcoms/zprofile",  require => Vcsrepo['zprezto'], }
  file { "/home/${user}/.zshenv":    ensure => link, target => "/home/${user}/.zprezto/runcoms/zshenv",    require => Vcsrepo['zprezto'], }

  # Make sure .zshrc has key bindings for the numpad.
  # One at a time so puppet can only append the line if it is not already in the file.
  # Confirm key codes by pressing <ctrl>+v followed by the key in question
  # http://superuser.com/questions/742171/zsh-z-shell-numpad-numlock-doesnt-work
  file { "/home/${user}/.zshrc":     ensure => link, target => "/home/${user}/.zprezto/runcoms/zshrc",     require => Vcsrepo['zprezto'], } ->
  file_line{ 'zshrc_numpad00': path => "/home/${user}/.zshrc", line => '# Keypad'} ->
  file_line{ 'zshrc_numpad01': path => "/home/${user}/.zshrc", line => '# 0 . Enter'} ->
  file_line{ 'zshrc_numpad02': path => "/home/${user}/.zshrc", line => 'bindkey -s "^[[2~" "0"'} ->
  file_line{ 'zshrc_numpad03': path => "/home/${user}/.zshrc", line => '#bindkey -s "^[[3~" "."'} ->
  file_line{ 'zshrc_numpad04': path => "/home/${user}/.zshrc", line => 'bindkey -s "^[OM" "^M"'} ->
  file_line{ 'zshrc_numpad05': path => "/home/${user}/.zshrc", line => '# 1 2 3'} ->
  file_line{ 'zshrc_numpad06': path => "/home/${user}/.zshrc", line => '#bindkey -s "^[OF" "1"'} ->
  file_line{ 'zshrc_numpad07': path => "/home/${user}/.zshrc", line => '#bindkey -s "^[OB" "2"'} ->
  file_line{ 'zshrc_numpad08': path => "/home/${user}/.zshrc", line => '#bindkey -s "^[[6~" "3"'} ->
  file_line{ 'zshrc_numpad09': path => "/home/${user}/.zshrc", line => '# 4 5 6'} ->
  file_line{ 'zshrc_numpad10': path => "/home/${user}/.zshrc", line => '#bindkey -s "^[OD" "4"'} ->
  file_line{ 'zshrc_numpad11': path => "/home/${user}/.zshrc", line => '#bindkey -s "^[OE" "5"'} ->
  file_line{ 'zshrc_numpad12': path => "/home/${user}/.zshrc", line => '#bindkey -s "^[OC" "6"'} ->
  file_line{ 'zshrc_numpad13': path => "/home/${user}/.zshrc", line => '# 7 8 9'} ->
  file_line{ 'zshrc_numpad14': path => "/home/${user}/.zshrc", line => '#bindkey -s "^[OH" "7"'} ->
  file_line{ 'zshrc_numpad15': path => "/home/${user}/.zshrc", line => '#bindkey -s "^[OA" "8"'} ->
  file_line{ 'zshrc_numpad16': path => "/home/${user}/.zshrc", line => '#bindkey -s "^[[5~" "9"'} ->
  file_line{ 'zshrc_numpad17': path => "/home/${user}/.zshrc", line => '# + - * /'} ->
  file_line{ 'zshrc_numpad18': path => "/home/${user}/.zshrc", line => 'bindkey -s "^[Ok" "+"'} ->
  file_line{ 'zshrc_numpad19': path => "/home/${user}/.zshrc", line => 'bindkey -s "^[Om" "-"'} ->
  file_line{ 'zshrc_numpad20': path => "/home/${user}/.zshrc", line => 'bindkey -s "^[Oj" "*"'} ->
  file_line{ 'zshrc_numpad21': path => "/home/${user}/.zshrc", line => 'bindkey -s "^[Oo" "/"'}

  # Modify our path so ccache is in it
  file_line{ 'path_ccache': path => "/home/${user}/.zshrc", line => 'export PATH=/usr/lib/ccache:$PATH'}

  # Enable the git module for zprezto
  file_line { 'zprezto_modules':
    ensure => present,
    path   => "/home/${user}/.zpreztorc",
    match  => '  \'prompt\'',
    line   => "  \'git\' \'command-not-found\' \'prompt\'",
  }

  # SSH KEYS FOR THE VM
  file { 'vm_private_key':
    path => "/home/${user}/.ssh/id_rsa",
    ensure => present,
    source => 'puppet:///modules/user_tools/id_rsa',
    owner => "${user}",
    mode => '600',
    replace => true,
  }

  file { 'vm_public_key':
    path => "/home/${user}/.ssh/id_rsa.pub",
    ensure => present,
    source => 'puppet:///modules/user_tools/id_rsa.pub',
    owner => "${user}",
    replace => true,
  }

  # SSH CONFIG FOR THE VM
  file { 'ssh_config':
    path => "/home/${user}/.ssh/config",
    ensure => present,
    source => 'puppet:///modules/user_tools/ssh_config',
    owner => "${user}",
    mode => '600',
    replace => true,
  }

  # SETUP ENVIRONMENT VARIABLES FOR SHELLS
  # This file does not need execute permissions (it is "sourced" not "executed")
  file { '/etc/profile.d/toolchain_init.sh':
    ensure => present,
    mode => '644',
    source => 'puppet:///modules/user_tools/toolchain_init.sh',
    replace => true,
  }

  # SETUP ROBOT HOSTS (note this isn't a file so we can use variables in here)
  file { '/etc/hosts':
    ensure  => present,
    mode    => '644',
    replace => true,
    content =>
"
127.0.0.1 $hostname.nubots.net  $hostname
127.0.0.1 localhost
127.0.1.1 ${user}
::1     localhost ip6-localhost ip6-loopback
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters

# Add robot hosts
10.1.1.1 d1 darwin1
10.1.1.2 d2 darwin2
10.1.1.3 d3 darwin3
10.1.1.4 d4 darwin4
10.1.1.5 d5 darwin5
10.1.1.6 d6 darwin6

10.1.1.11 i1 igus1
10.1.1.12 i2 igus2
",
  }
}
