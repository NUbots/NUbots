class dev_tools {

  # Update apt before getting any packages (if we need to)
  exec { "apt-update":
    command => "/usr/bin/apt-key update && /usr/bin/apt-get update",
    onlyif => "/bin/sh -c '[ ! -f /var/cache/apt/pkgcache.bin ] || /usr/bin/find /etc/apt/* -cnewer /var/cache/apt/pkgcache.bin | /bin/grep . > /dev/null'",
  } ->
  exec { "install-software-properties":
    command => "/usr/bin/apt-get install -y software-properties-common",
    unless => '/usr/bin/dpkg -s software-properties-common',
  } ->
  apt::ppa {'ppa:ubuntu-toolchain-r/test': } ~>
  exec { "apt-update-ppa":
    command => "/usr/bin/apt-get update",
    refreshonly => true
  } -> Package <| |>

  # Tools
  package { 'vim': ensure => latest, }
  package { 'screen': ensure => latest, }
  package { 'zsh': ensure => latest, }
  package { 'htop': ensure => latest, }
  package { 'gdb': ensure => latest, }
  package { 'cmake': ensure => latest, }
  package { 'cmake-curses-gui': ensure => latest, }
  package { 'linux-headers-generic': ensure => latest, }
  package { 'dos2unix': ensure => latest, }
  package { 'unzip': ensure => latest, }
  package { 'rsync': ensure => latest, }
  package { 'git': ensure => latest, }
  package { 'graphviz': ensure => latest, }
  package { 'build-essential': ensure => latest, }
  package { 'libncurses5-dev:amd64': ensure => latest, }
  package { 'libncurses5-dev:i386': ensure => latest, }
  package { 'gcc-6': name => 'gcc-6-multilib', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'g++-6': name => 'g++-6-multilib', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'gfortran-6': name => 'gfortran-6-multilib', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'binutils': name => 'binutils-multiarch', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'binutils-dev': name => 'binutils-multiarch-dev', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'ninja-build': ensure => latest, }
  package { 'nasm': ensure => latest, }
  package { 'libusb-1.0-0:amd64': ensure => latest, }
  package { 'libusb-1.0-0:i386': ensure => latest, }
  package { 'libusb-1.0-0-dev:amd64': ensure => latest, }
  package { 'libusb-1.0-0-dev:i386': ensure => latest, }
  package { 'autopoint': ensure => latest, }
  package { 'gettext': ensure => latest, }
  package { 'python3-pip': ensure => latest, }

  # System libraries
  package { 'libasound2-dev:amd64': ensure => latest, }
  package { 'libasound2-dev:i386': ensure => latest, }

  # For ELLCC
  package { 'subversion': ensure => latest, }
  package { 'texinfo': ensure => latest, }
  package { 'flex': ensure => latest, }
  package { 'pkg-config': ensure => latest, }
  package { 'zlib1g-dev': ensure => latest, }
  package { 'autoconf': ensure => latest, }
  package { 'libtool': ensure => latest, }
  package { 'libglib2.0-dev': ensure => latest, }

  # Set the vagrant shell to zsh
  user { 'vagrant': shell => '/bin/zsh', require => Package['zsh'], }

  # Setup prezto for a richer zsh experience
  vcsrepo { 'zprezto':
    ensure   => present,
    path     => '/home/vagrant/.zprezto',
    provider => git,
    owner    => 'vagrant',
    group    => 'vagrant',
    source   => 'https://github.com/sorin-ionescu/prezto.git',
    require  => Package['git'],
  }

  # Create the required links for zprezto
  file { '/home/vagrant/.zlogin':    ensure => link, target => '/home/vagrant/.zprezto/runcoms/zlogin',    require => Vcsrepo['zprezto'], }
  file { '/home/vagrant/.zlogout':   ensure => link, target => '/home/vagrant/.zprezto/runcoms/zlogout',   require => Vcsrepo['zprezto'], }
  file { '/home/vagrant/.zpreztorc': ensure => link, target => '/home/vagrant/.zprezto/runcoms/zpreztorc', require => Vcsrepo['zprezto'], }
  file { '/home/vagrant/.zprofile':  ensure => link, target => '/home/vagrant/.zprezto/runcoms/zprofile',  require => Vcsrepo['zprezto'], }
  file { '/home/vagrant/.zshenv':    ensure => link, target => '/home/vagrant/.zprezto/runcoms/zshenv',    require => Vcsrepo['zprezto'], }

  # Make sure .zshrc has key bindings for the numpad.
  # One at a time so puppet can only append the line if it is not already in the file.
  # Confirm key codes by pressing <ctrl>+v followed by the key in question
  # http://superuser.com/questions/742171/zsh-z-shell-numpad-numlock-doesnt-work
  file { '/home/vagrant/.zshrc':     ensure => link, target => '/home/vagrant/.zprezto/runcoms/zshrc',     require => Vcsrepo['zprezto'], } ->
  file_line{ 'zshrc_numpad00': path => '/home/vagrant/.zshrc', line => '# Keypad'} ->
  file_line{ 'zshrc_numpad01': path => '/home/vagrant/.zshrc', line => '# 0 . Enter'} ->
  file_line{ 'zshrc_numpad02': path => '/home/vagrant/.zshrc', line => 'bindkey -s "^[[2~" "0"'} ->
  file_line{ 'zshrc_numpad03': path => '/home/vagrant/.zshrc', line => '#bindkey -s "^[[3~" "."'} ->
  file_line{ 'zshrc_numpad04': path => '/home/vagrant/.zshrc', line => 'bindkey -s "^[OM" "^M"'} ->
  file_line{ 'zshrc_numpad05': path => '/home/vagrant/.zshrc', line => '# 1 2 3'} ->
  file_line{ 'zshrc_numpad06': path => '/home/vagrant/.zshrc', line => '#bindkey -s "^[OF" "1"'} ->
  file_line{ 'zshrc_numpad07': path => '/home/vagrant/.zshrc', line => '#bindkey -s "^[OB" "2"'} ->
  file_line{ 'zshrc_numpad08': path => '/home/vagrant/.zshrc', line => '#bindkey -s "^[[6~" "3"'} ->
  file_line{ 'zshrc_numpad09': path => '/home/vagrant/.zshrc', line => '# 4 5 6'} ->
  file_line{ 'zshrc_numpad10': path => '/home/vagrant/.zshrc', line => '#bindkey -s "^[OD" "4"'} ->
  file_line{ 'zshrc_numpad11': path => '/home/vagrant/.zshrc', line => '#bindkey -s "^[OE" "5"'} ->
  file_line{ 'zshrc_numpad12': path => '/home/vagrant/.zshrc', line => '#bindkey -s "^[OC" "6"'} ->
  file_line{ 'zshrc_numpad13': path => '/home/vagrant/.zshrc', line => '# 7 8 9'} ->
  file_line{ 'zshrc_numpad14': path => '/home/vagrant/.zshrc', line => '#bindkey -s "^[OH" "7"'} ->
  file_line{ 'zshrc_numpad15': path => '/home/vagrant/.zshrc', line => '#bindkey -s "^[OA" "8"'} ->
  file_line{ 'zshrc_numpad16': path => '/home/vagrant/.zshrc', line => '#bindkey -s "^[[5~" "9"'} ->
  file_line{ 'zshrc_numpad17': path => '/home/vagrant/.zshrc', line => '# + - * /'} ->
  file_line{ 'zshrc_numpad18': path => '/home/vagrant/.zshrc', line => 'bindkey -s "^[Ok" "+"'} ->
  file_line{ 'zshrc_numpad19': path => '/home/vagrant/.zshrc', line => 'bindkey -s "^[Om" "-"'} ->
  file_line{ 'zshrc_numpad20': path => '/home/vagrant/.zshrc', line => 'bindkey -s "^[Oj" "*"'} ->
  file_line{ 'zshrc_numpad21': path => '/home/vagrant/.zshrc', line => 'bindkey -s "^[Oo" "/"'}

  # INSTALL PYTHON PACKAGES (we need python-pip to use the pip provider)
  Package['python3-pip'] -> Package <| provider == 'pip3' |>
  package { 'pyparsing': ensure => installed, provider => 'pip3' }
  package { 'pydotplus': ensure => installed, provider => 'pip3' }
  package { 'pygments':  ensure => installed, provider => 'pip3' }
  package { 'termcolor': ensure => installed, provider => 'pip3' }
  package { 'protobuf':  ensure => installed, provider => 'pip3' }
  package { 'mmh3':      ensure => installed, provider => 'pip3' }
  # python::pip { 'pybfd': ensure => latest }#, url => 'https://github.com/Groundworkstech/pybfd/archive/master.tar.gz' }

  # Enable the git module for zprezto
  file_line { 'zprezto_modules':
    ensure => present,
    path   => '/home/vagrant/.zpreztorc',
    match  => '  \'prompt\'',
    line   => "  \'git\' \'command-not-found\' \'prompt\'",
  }

  # SSH KEYS FOR THE VM
  file { 'vm_private_key':
    path => '/home/vagrant/.ssh/id_rsa',
    ensure => present,
    source => 'puppet:///modules/dev_tools/id_rsa',
    owner => 'vagrant',
    mode => '600', 
    replace => true,
  }

  file { 'vm_public_key':
    path => '/home/vagrant/.ssh/id_rsa.pub',
    ensure => present,
    source => 'puppet:///modules/dev_tools/id_rsa.pub',
    owner => 'vagrant', 
    replace => true,
  }

  # SSH CONFIG FOR THE VM
  file { 'ssh_config':
    path => '/home/vagrant/.ssh/config',
    ensure => present,
    source => 'puppet:///modules/dev_tools/ssh_config',
    owner => 'vagrant',
    mode => '600', 
    replace => true,
  }

  # SETUP ENVIRONMENT VARIABLES FOR SHELLS
  # This file does not need execute permissions (it is "sourced" not "executed")
  file { '/etc/profile.d/toolchain_init.sh':
    ensure => present,
    mode => '644',
    source => 'puppet:///modules/dev_tools/toolchain_init.sh', 
    replace => true,
  }

  # SETUP ROBOT HOSTS
  file { '/etc/hosts':
    ensure => present,
    mode => '644',
    source => 'puppet:///modules/dev_tools/hosts', 
    replace => true,
  }

  # INSTALL ROBOT HOST PARSER
  file { '/nubots/toolchain/find_robot_hosts.sh':
    ensure => present,
    mode => '755',
    source => 'puppet:///modules/dev_tools/find_robot_hosts.sh', 
    replace => true,
  }

  # SETUP OUR ALTERNATIVES SO WE USE THE CORRECT COMPILER
  exec {'fix_compiler_environment':
    command => 'update-alternatives --remove-all gcc \
             ;  update-alternatives --remove-all g++ \
             ;  update-alternatives --remove-all gfortan \
             ;  update-alternatives --install /usr/bin/ld ld /usr/bin/ld.bfd 10 \
             && update-alternatives --install /usr/bin/ld ld /usr/bin/ld.gold 20 \
             && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 100 \
                                    --slave /usr/bin/g++ g++ /usr/bin/g++-6 \
                                    --slave /usr/bin/gfortran gfortran /usr/bin/gfortran-6',
    require => [ Package['gcc-6'], Package['g++-6'], Package['gfortran-6'], Package['build-essential'], Package['binutils'], ]
  }
}
