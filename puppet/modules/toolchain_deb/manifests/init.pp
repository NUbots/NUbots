class toolchain_deb {

  $build_dir = '/home/vagrant/nubots-toolchain'
  $excludes = '--exclude=/nubots/toolchain/src/ --exclude=/nubots/toolchain/native/src/ --exclude=/nubots/toolchain/nuc7i7bnh/src/ '

  file { [ "${build_dir}" , "${build_dir}/DEBIAN", ]:
    ensure  => directory,
  } ->
  exec { "copy-toolchain":
    command => "rsync -a ${excludes} /nubots .",
    cwd     => "${build_dir}",
  } ->
  file { 'toolchain_control':
    ensure  => present,
    path    => "${build_dir}/DEBIAN/control",
    source  => 'puppet:///modules/toolchain_deb/control',
  } ->
  exec { "build_nubots_deb":
    command => "/usr/bin/dpkg-deb --build ${build_dir}",
    cwd     => "/home/vagrant",
    timeout => 0,
    creates => '/home/vagrant/nubots-toolchain.deb',
  }
}
