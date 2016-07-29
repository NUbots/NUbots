
class installer::prerequisites (Hash $archs = {'DarwinOp' => {'abi' => 32, 'args' => '-march=bonnell -mtune=bonnell -m32 -mno-movbe -mfxsr -mmmx -msahf -msse -msse2 -msse3 -mssse3 --param l1-cache-size=24 --param l1-cache-line-size=64 --param l2-cache-size=512'}}) {

  # Make the generic directories.
  file { [ '/nubots',
           '/nubots/toolchain',
           '/nubots/toolchain/bin',
           '/nubots/toolchain/include',
           '/nubots/toolchain/src' ]:
    ensure => directory,
  } ->
  file { 'install_from_source':
    path => '/nubots/toolchain/bin/install_from_source',
    ensure => file,
    mode => '755',
    source => 'puppet:///modules/installer/install_from_source',
  }

  # Make the architecture specific directories.
  $archs.each |String $arch, Hash $params| {
    file { [ "/nubots/toolchain/${arch}",
             "/nubots/toolchain/${arch}/bin",
             "/nubots/toolchain/${arch}/include",
             "/nubots/toolchain/${arch}/lib${abi}",
             "/nubots/toolchain/${arch}/man",
             "/nubots/toolchain/${arch}/share",
             "/nubots/toolchain/${arch}/src" ]:
      ensure => directory,
    }
  }
}

define installer (
  $url,
  $creates = '',
  $args = '',
  $environment = [],
  $src_dir = '',
  $method = 'auto',
  $prefix = '/nubots/toolchain',
  $prebuild = 'echo', # Colon is a noop
  $postbuild = 'echo', # Colon is a noop
  $strip_components = 1,
  Hash $archs = $archs,
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

  $archs.each |String $arch, Hash $params| {
    if $params['abi'] == 64 {
      $abi = $params['abi']
    }

    else {
      $abi = ''
    }

    $args = $params['args']

    if $creates == '' {
      $create = "${prefix}/${arch}/lib${abi}/lib${name}.a"
    }

    else {
      $create = "${prefix}/${arch}/lib${abi}/${creates}"
    }

    case $extension {
      'h', 'hpp': {
        # Download and put in include
        $basename = basename($url)

        wget::fetch { "${arch}_${name}":
          destination => "${prefix}/${arch}/include/${basename}",
          source => $url,
        }
      }
      default: {
        # Download and install
        archive { "${arch}_${name}":
          url    => $url,
          target => "/nubots/toolchain/${arch}/src/${name}",
          src_target => "/nubots/toolchain/${arch}/src",
          purge_target => true,
          checksum => false,
          follow_redirects => true,
          timeout => 0,
          extension => $extension,
          strip_components => $strip_components,
        } ~>
        exec { "install_${arch}_${name}":
          command => "${prebuild} ;
                      install_from_source '${prefix}' '${method}' '${args}' ;
                      ${postbuild} ;",
          creates => "${create}",
          cwd => "/nubots/toolchain/${arch}/src/${name}/${src_dir}",
          environment => $environment,
          path =>  [  "/nubots/toolchain/${arch}/bin", '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
          timeout => 0,
          refreshonly => true,
          require => [
            Class['installer::prerequisites'],
            Archive["${arch}_${name}"],
            Class['dev_tools'],
          ],
        }
      }
    }
  }



}
