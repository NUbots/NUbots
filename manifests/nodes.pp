class vim {
  package { 'vim':
    ensure => latest
  } ->
  package { 'vim-puppet':  # syntax highlighting
    ensure => latest,
  } ->
  file { ['/home/mitchell/.vim', '/home/mitchell/.vim/plugin']:
    ensure => directory,
  } ->
  file { '/home/mitchell/.vim/plugin/puppet.vim':
    ensure => link,
    target => '/usr/share/vim/addons/syntax/puppet.vim',
  }
}

class gcc48 {
  include apt

  apt::ppa { 'ppa:ubuntu-toolchain-r/test': }

  package { 'gcc-4.8':
    ensure => latest,
    require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'], 
  } ->
  exec { 'gcc-4.8_alternatives':
    command => 'update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.6',
    path => $path,
    refreshonly => true, 
  }

  package { 'g++-4.8':
    ensure => latest,
    require => Apt::Ppa['ppa:ubuntu-toolchain-r/test'], 
  } ->
  exec { 'g++-4.8_alternatives':
    command => 'update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 80 --slave /usr/bin/g++ g++ /usr/bin/g++-4.8',
    path => $path,
    refreshonly => true, 
  }
}

class catch {
  wget::fetch { 'catch.hpp':
    destination => '/usr/local/include/catch.hpp',
    source => 'https://raw.github.com/philsquared/Catch/master/single_include/catch.hpp',
  }
}

class nuclear {
  include gcc48
  include catch

  $nuclear_build_dir = '/home/mitchell/NUbots/NUClear/build'

  file { 'nubots_dir':
    path => '/home/mitchell/NUbots',
    ensure => directory,
  } ->
  vcsrepo { 'nuclear_repo':
    path => '/home/mitchell/NUbots/NUClear',
    ensure => present,
    provider => 'git',
    source => "https://github.com/Fastcode/NUClear.git",
    require => Package['git'],
  } ->
  file { 'nuclear_build_dir':
    path => $nuclear_build_dir,
    ensure => directory,
    purge => true,
    force => true,
  } ~>
  exec { 'nuclear_cmake':
    require => [Package['cmake'], Package['libzmq-dev'], Package['protobuf-compiler']],
    command => 'cmake ..',
    cwd => $nuclear_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
  } ~>
  exec { 'nuclear_make':
    command => 'make -j',
    cwd => $nuclear_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
  } ~>
  exec { 'nuclear_install':
    command => 'make install',
    cwd => $nuclear_build_dir,
    path => $path,
    refreshonly => true,
    logoutput => "on_failure",
  }
}

class nuclearport {
  require 'nuclear'
}

node default {
  package { 'cmake': ensure => latest }
  package { 'git': ensure => latest }
  package { 'openssh-server': ensure => latest }
  package { 'libzmq-dev': ensure => latest }
  package { ['libprotobuf-dev', 'protobuf-compiler']: ensure => latest }
  include vim
  include nuclearPort
}
