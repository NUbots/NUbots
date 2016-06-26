class dev_tools {

  # Update apt before getting any packages (if we need to)
  exec { "apt-update":
    command => "/usr/bin/apt-get update",
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
  package { 'cmake-curses-gui': ensure => latest, }
  package { 'linux-headers-generic': ensure => latest, }
  package { 'dos2unix': ensure => latest, }
  package { 'unzip': ensure => latest, }
  package { 'rsync': ensure => latest, }
  package { 'git': ensure => latest, }
  package { 'graphviz': ensure => latest, }
  package { 'build-essential': ensure => latest, }
  package { 'python-dev': ensure => latest, }
  package { 'python-pip': ensure => latest, }
  package { 'libncurses5-dev': ensure => latest, }
  package { 'gcc-6': ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'g++-6': ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'gfortran-6': ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'binutils': ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'binutils-dev': ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'ninja-build': ensure => latest, }
  package { 'yasm': ensure => latest, }

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
  file { '/home/vagrant/.zshrc':     ensure => link, target => '/home/vagrant/.zprezto/runcoms/zshrc',     require => Vcsrepo['zprezto'], }

  # Enable the git module for zprezto
  file_line { 'zprezto_modules':
    ensure => present,
    path   => '/home/vagrant/.zpreztorc',
    match  => '  \'prompt\'',
    line   => "  \'git\' \'command-not-found\' \'prompt\'",
  }

  # Load environment variables for the toolchain in zsh
  file_line { 'zsh_toolchain_environment':
    ensure => present,
    path   => '/home/vagrant/.zshrc',
    line   => 'source /etc/profile.d/toolchain_init.sh',
  }

  # System libraries
  package { 'libasound2-dev': ensure => latest, }

  # INSTALL PYTHON PACKAGES (we need python-pip to use the pip provider)
  Package['python-pip'] -> Package <| provider == 'pip' |>
  package { 'pyparsing': ensure => installed, provider => 'pip' }
  package { 'pydotplus': ensure => installed, provider => 'pip' }
  package { 'pygments': ensure => installed, provider => 'pip' }
  # python::pip { 'pybfd': ensure => latest }#, url => 'https://github.com/Groundworkstech/pybfd/archive/master.tar.gz' }

  # SSH KEYS FOR THE VM
  file { 'vm_private_key':
      path => '/home/vagrant/.ssh/id_rsa',
      ensure => present,
      source => 'puppet:///modules/dev_tools/id_rsa',
      owner => 'vagrant',
      mode => '600', }

  file { 'vm_public_key':
      path => '/home/vagrant/.ssh/id_rsa.pub',
      ensure => present,
      source => 'puppet:///modules/dev_tools/id_rsa.pub',
      owner => 'vagrant', }

  # SSH CONFIG FOR THE VM
  file { 'ssh_config':
      path => '/home/vagrant/.ssh/config',
      ensure => present,
      source => 'puppet:///modules/dev_tools/ssh_config',
      owner => 'vagrant',
      mode => '600', }

  # SETUP ENVIRONMENT VARIABLES FOR SHELLS
  file { '/etc/profile.d/toolchain_init.sh':
    ensure => present,
    mode => '755',
    source => 'puppet:///modules/dev_tools/toolchain_init.sh', }

  # SETUP OUR ALTERNATIVES SO WE USE THE CORRECT COMPILER
  exec {'fix_compiler_environment':
    command => 'update-alternatives --remove-all gcc \
             ;  update-alternatives --install /usr/bin/ld ld /usr/bin/ld.bfd 10 \
             && update-alternatives --install /usr/bin/ld ld /usr/bin/ld.gold 20 \
             && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 100 \
                                    --slave /usr/bin/g++ g++ /usr/bin/g++-6 \
                                    --slave /usr/bin/gfortran gfortran /usr/bin/gfortran-6',
    require => [ Package['gcc-6'], Package['g++-6'], Package['gfortran-6'], Package['build-essential'], Package['binutils'] ]
  }
}
