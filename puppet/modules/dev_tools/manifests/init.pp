class dev_tools {

  # Update apt before getting any packages
  exec { "apt-update":
    command => "/usr/bin/apt-get update"
  } ->
  exec { "install-software-properties":
    command => "/usr/bin/apt-get install -y software-properties-common"
  } ->
  apt::ppa {'ppa:ubuntu-toolchain-r/test': } ->
  exec { "apt-update-ppa":
    command => "/usr/bin/apt-get update"
  } -> Package <| |>

  package { 'vim': ensure => latest, }
  package { 'screen': ensure => latest, }
  package { 'htop': ensure => latest, }
  package { 'gdb': ensure => latest, }
  package { 'cmake-curses-gui': ensure => latest, }
  package { 'linux-headers-generic': ensure => latest, }
  package { 'dos2unix': ensure => latest, }
  package { 'rsync': ensure => latest, }
  package { 'build-essential': ensure => latest, }
  package { 'gcc-5': ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'g++-5': ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'gfortran-5': ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'binutils': ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'ninja-build': ensure => latest, }
  package { 'yasm': ensure => latest, }
  package { 'libboost-dev': ensure => latest, }

  file { '/etc/profile.d/toolchain_init.sh':
    ensure => present,
    mode => '755',
    source => 'puppet:///modules/dev_tools/toolchain_init.sh', }

  file { '/usr/bin/ar_bin':
    ensure => present,
    mode => '755',
    source => '/usr/bin/ar',
    subscribe => Package['binutils'], } ~>
  file { '/usr/bin/ar':
    ensure => present,
    mode => '755',
    source => 'puppet:///modules/dev_tools/ar', }

  file { '/usr/bin/nm_bin':
    ensure => present,
    mode => '755',
    source => '/usr/bin/nm',
    subscribe => Package['binutils'],  } ~>
  file { '/usr/bin/nm':
    ensure => present,
    mode => '755',
    source => 'puppet:///modules/dev_tools/nm', }

  file { '/usr/bin/ranlib_bin':
    ensure => present,
    mode => '755',
    source => '/usr/bin/ranlib',
    subscribe => Package['binutils'],  } ~>
  file { '/usr/bin/ranlib':
    ensure => present,
    mode => '755',
    source => 'puppet:///modules/dev_tools/ranlib', }

  exec {'fix_compiler_environment':
    command => 'update-alternatives --install /usr/bin/ld ld /usr/bin/ld.bfd 10 \
             && update-alternatives --install /usr/bin/ld ld /usr/bin/ld.gold 20 \
             && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 100 \
                                    --slave /usr/bin/g++ g++ /usr/bin/g++-5 \
                                    --slave /usr/bin/gfortran gfortran /usr/bin/gfortran-5',
    require => [ Package['gcc-5'], Package['gfortran-5'], Package['build-essential'], Package['binutils'] ]
  }
}