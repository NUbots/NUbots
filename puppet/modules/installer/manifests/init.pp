
class installer::prerequisites {

  file { [ '/nubots',
           '/nubots/toolchain',
           '/nubots/toolchain/bin',
           '/nubots/toolchain/include',
           '/nubots/toolchain/lib',
           '/nubots/toolchain/man',
           '/nubots/toolchain/share',
           '/nubots/toolchain/src' ]:
    ensure => directory,
  } ->
  file { 'install_from_source':
    path => '/nubots/toolchain/bin/install_from_source',
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
  $src_dir = '',
  $method = 'auto',
  $prefix = '/nubots/toolchain',
  $prebuild = 'echo', # Colon is a noop
  $postbuild = 'echo', # Colon is a noop
  $strip_components = 1,
  $creates = "/nubots/toolchain/lib/lib${name}.a"
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
    /.*\.h/         => 'h',
    /.*\.hpp/       => 'hpp',
    default         => 'UNKNOWN',
  }

  case $extension {
    'h', 'hpp': {
      # Download and put in include
      $basename = basename($url)

      wget::fetch { "${name}":
        destination => "${prefix}/include/${basename}",
        source => $url,
      }
    }
    default: {
      # Download and install
      archive { "${name}":
        url    => $url,
        target => "/nubots/toolchain/src/${name}",
        src_target => '/nubots/toolchain/src',
        purge_target => true,
        checksum => false,
        follow_redirects => true,
        timeout => 0,
        extension => $extension,
        strip_components => $strip_components,
      } ~>
      exec { "install_${name}":
        command => "${prebuild} ;
                    install_from_source '${prefix}' '${method}' ${lto} ${args} ;
                    ${postbuild} ;",
        creates => $creates,
        cwd => "/nubots/toolchain/src/${name}/${src_dir}",
        environment => $environment,
        path =>  [  '/nubots/toolchain/bin', '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
        timeout => 0,
        refreshonly => true,
        require => [
          Class['installer::prerequisites'],
          Archive["${name}"],
          Class['dev_tools'],
        ],
      }
    }
  }

}