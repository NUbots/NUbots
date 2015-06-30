
class installer::prerequisites {
  file { 'install_from_source':
    path => '/usr/local/bin/install_from_source',
    ensure => file,
    mode => '755',
    source => 'puppet:///modules/installer/install_from_source',
  }
}

define installer (
  $url,
  $args = '',
  $environment = [],
  $lto = true,
  $method = 'auto',
  $prefix = '/nubots/toolchain',
  $strip_components = 1
) {

  $extension = $url ? {
    /.*\.zip/       => 'zip',
    /.*\.tgz/       => 'tgz',
    /.*\.tar\.gz/   => 'tar.gz',
    /.*\.txz/       => 'txz',
    /.*\.tar\.xz/   => 'tar.xz',
    /.*\.tbz/       => 'tbz',
    /.*\.tbz2/      => 'tbz2',
    /.*\.tar\.bz2/  => 'tar.bz2',
    default         => 'UNKNOWN',
  }

  # Download the URL and extract
  archive { "${name}":
    url    => $url,
    target => '/nubots/toolchain/src',
    checksum => false,
    extension => $extension,
    strip_components => $strip_components,
  } ~>
  exec { "install_${name}":
    command => "/usr/local/bin/install_from_source '${prefix}' '${method}' ${lto} ${args}",
    creates => "/nubots/toolchain/lib/lib${name}.a",
    cwd => "/nubots/toolchain/src/${name}",
    environment => $environment,
    path =>  [  '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
    timeout => 0,
    refreshonly => true,
    require => [
      File['install_from_source'],
      Archive["${name}"],
      Exec['fix_compiler_environment'],
    ],
  }

    # Work out if the source is autotools (has ./configure) cmake (has CMakeLists) or make (has Makefile)

    # Execute the correct ones configuration stage

    # Execute the make stage

    # Execute the make install stage

}