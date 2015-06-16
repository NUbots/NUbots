# == Class: cppformat
class cppformat {

  $cppformat_dir = "/tmp/cppformat"
  $cppformat_build_dir = "$cppformat_dir/build"

  vcsrepo { 'cppformat_repo':
    path => "$cppformat_dir",
    source => "https://github.com/cppformat/cppformat.git",
    depth => 1,
    provider => 'git',
    ensure => present
  } ->
  file { 'cppformat_build_dir':
    path => $cppformat_build_dir,
    ensure => directory,
    purge => true,
    force => true
  } ~>
  exec { 'cppformat_cmake':
    command => 'cmake .. -DCMAKE_CXX_FLAGS=-O3 -DCMAKE_C_FLAGS=-O3',
    cwd => $cppformat_build_dir,
    refreshonly => true,
    logoutput => "on_failure"
  } ~>
  exec { 'cppformat_make':
    command => 'make -j',
    cwd => $cppformat_build_dir,
    refreshonly => true,
    logoutput => "on_failure"
  } ~>
  exec { 'cppformat_makeinstall':
    command => 'make install',
    cwd => $cppformat_build_dir,
    refreshonly => true,
    logoutput => "on_failure"
  }
}
