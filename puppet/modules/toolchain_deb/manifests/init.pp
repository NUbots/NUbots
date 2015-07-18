class toolchain_deb {

  $build_dir = '/home/vagrant/nubots-toolchain'

  file { "${build_dir}":
    ensure => directory
  } ->
  exec { "copy-toolchain":
    command => "cp -r /nubots .",
    cwd => "${build_dir}",
  } ->
  exec { "remove-toolchain-src":
    command => "rm -rf src",
    cwd => "${build_dir}/nubots/toolchain",
  } ->
  file { "${build_dir}/DEBIAN":
    ensure => directory
  } ->
  file { 'toolchain_control':
    ensure => present,
    path => "${build_dir}/DEBIAN/control",
    source => 'puppet:///modules/toolchain_deb/control',
  } ->
  exec { "build_nubots_deb":
    command => "/usr/bin/dpkg-deb --build ${build_dir}",
    cwd => "/home/vagrant",
    timeout => 0,
  }
}