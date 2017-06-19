class build_tools {

  # Update apt before getting any packages (if we need to)
  exec { "apt-update":
    command => "/usr/bin/apt-key update && /usr/bin/apt-get update",
    onlyif => "/bin/sh -c '[ ! -f /var/cache/apt/pkgcache.bin ] || /usr/bin/find /etc/apt/* -cnewer /var/cache/apt/pkgcache.bin | /bin/grep . > /dev/null'",
  } ->
  # We also need to ensure software-properties is installed for apt-add-repository
  exec { "install-software-properties":
    command => "/usr/bin/apt-get install -y software-properties-common",
    unless => '/usr/bin/dpkg -s software-properties-common',
  } ->
  # Add the ubuntu test toolchain ppa (for modern g++ etc)
  apt::ppa {'ppa:ubuntu-toolchain-r/test': } ~>
  exec { "apt-update-ppa":
    command => "/usr/bin/apt-get update",
    refreshonly => true
  } -> Package <| |>

  # Tools
  package { 'cmake': ensure => latest, }
  package { 'linux-headers-generic': ensure => latest, }
  package { 'rsync': ensure => latest, }
  package { 'git': ensure => latest, }
  package { 'build-essential': ensure => latest, }
  package { 'libncurses5-dev:amd64': ensure => latest, require => [ Package['gcc-7'], Package['g++-7'], ], }
  package { 'libncurses5-dev:i386': ensure => latest, require => [ Package['gcc-7'], Package['g++-7'], ], }
  package { 'gcc-7': name => 'gcc-7-multilib', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'g++-7': name => 'g++-7-multilib', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
  package { 'gfortran-7': name => 'gfortran-7-multilib', ensure => latest, require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'] }
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
  package { 'python-pip': ensure => latest, }

  # CM730 firmware compilation.
  package { 'gcc-arm-none-eabi': ensure => latest, }
  package { 'libnewlib-arm-none-eabi': ensure => latest, }
  package { 'gdb-arm-none-eabi': ensure => latest, install_options => [ {'-o' => 'Dpkg::Options::=--force-overwrite'}, ], }

  # System libraries
  package { 'libasound2-dev:amd64': ensure => latest, }
  package { 'libasound2-dev:i386': ensure => latest, }

  # INSTALL PYTHON PACKAGES (we need python-pip to use the pip provider)
  exec {'install_python3_packages':
    command => '/usr/bin/pip3 install pyparsing &&
                /usr/bin/pip3 install pydotplus &&
                /usr/bin/pip3 install pygments &&
                /usr/bin/pip3 install termcolor &&
                /usr/bin/pip3 install protobuf &&
                /usr/bin/pip3 install mmh3 &&
                /usr/bin/pip3 install numpy',
    require => [ Package['python3-pip'], ]
  }

    # INSTALL PYTHON PACKAGES (we need python-pip to use the pip provider)
  exec {'install_python_packages':
    command => '/usr/bin/pip install pyparsing &&
                /usr/bin/pip install pydotplus &&
                /usr/bin/pip install pygments &&
                /usr/bin/pip install termcolor &&
                /usr/bin/pip install protobuf &&
                /usr/bin/pip install mmh3 &&
                /usr/bin/pip install numpy',
    require => [ Package['python-pip'], ]
  }

  # SETUP OUR ALTERNATIVES SO WE USE THE CORRECT COMPILER
  exec {'fix_compiler_environment':
    command => '/usr/bin/update-alternatives --remove-all gcc \
             ;  /usr/bin/update-alternatives --remove-all g++ \
             ;  /usr/bin/update-alternatives --remove-all gfortan \
             ;  /usr/bin/update-alternatives --install /usr/bin/ld ld /usr/bin/ld.bfd 10 \
             && /usr/bin/update-alternatives --install /usr/bin/ld ld /usr/bin/ld.gold 20 \
             && /usr/bin/update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 100 \
                                             --slave /usr/bin/g++ g++ /usr/bin/g++-7 \
                                             --slave /usr/bin/gfortran gfortran /usr/bin/gfortran-7',
    require => [ Package['gcc-7'], Package['g++-7'], Package['gfortran-7'], Package['build-essential'], Package['binutils'], ]
  }
}
