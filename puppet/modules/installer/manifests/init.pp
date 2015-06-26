
class installer::prerequisites {
  file {'/usr/local/bin/install_from_source':
    ensure => file,
    mode => '755',
    source => 'puppet:///modules/installer/install_from_source',
  }

  exec {'fix_compiler_environment':
    command => "update-alternatives --install /usr/bin/ld ld /usr/bin/ld.bfd 10 \
             && update-alternatives --install /usr/bin/ld ld /usr/bin/ld.gold 20 \
             && mv /usr/bin/ar /usr/bin/ar_bin \
             && echo '#/bin/bash'                                                                    > /usr/bin/ar \
             && echo '/usr/bin/ar_bin $@ --plugin /usr/lib/gcc/i686-linux-gnu/4.9/liblto_plugin.so'     >> /usr/bin/ar \
             && chmod +x /usr/bin/ar \
             && mv /usr/bin/ranlib /usr/bin/ranlib_bin \
             && echo '#/bin/bash'                                                                    > /usr/bin/ranlib \
             && echo '/usr/bin/ranlib_bin $@ --plugin /usr/lib/gcc/i686-linux-gnu/4.9/liblto_plugin.so' >> /usr/bin/ranlib \
             && chmod +x /usr/bin/ranlib \
             && mv /usr/bin/nm /usr/bin/nm_bin \
             && echo '#/bin/bash'                                                                    > /usr/bin/nm \
             && echo '/usr/bin/nm_bin $@ --plugin /usr/lib/gcc/i686-linux-gnu/4.9/liblto_plugin.so'     >> /usr/bin/nm \
             && chmod +x /usr/bin/nm",
    creates => "/usr/bin/ar_bin",
  }
}

define installer (
  $url,
  $args = '',
  $environment = [],
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
  }

  exec { "install_${name}":
    command => "/usr/local/bin/install_from_source '${prefix}' '${method}' ${args}",
    creates => "/nubots/toolchain/lib/lib${name}.a",
    cwd => "/nubots/toolchain/src/${name}",
    environment => $environment,
    path =>  [ '/bin/', '/sbin/' , '/usr/bin/', '/usr/sbin/' ],
    timeout => 0,
    require => [
      File['/usr/local/bin/install_from_source'],
      Archive["${name}"]
    ],
  }

    # Work out if the source is autotools (has ./configure) cmake (has CMakeLists) or make (has Makefile)

    # Execute the correct ones configuration stage

    # Execute the make stage

    # Execute the make install stage

}