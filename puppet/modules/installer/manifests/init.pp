
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
    if $params['abi'] == 64 {
      $abi = $params['abi']
    }

    else {
      $abi = ''
    }

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
  $url = '',
  $creates = '',
  $args = [],
  $environment = [],
  $src_dir = '',
  $method = 'auto',
  $prefix = '/nubots/toolchain',
  $prebuild = 'echo', # Colon is a noop
  $postbuild = 'echo', # Colon is a noop
  $strip_components = 1,
  $extension = 'UNKNOWN',
  Hash $archs = $archs,
) {

  $archs.each |String $arch, Hash $params| {
    # Determine which ABI we are using.
    if $params['abi'] == 64 {
      $abi = $params['abi']
    }

    else {
      $abi = ''
    }

    # Ensure we have a valid "creates" variable.
    if $creates == '' {
      $create = "${prefix}/${arch}/lib${abi}/lib${name}.a"
    }

    else {
      $create = "${prefix}/${arch}/lib${abi}/${creates}"
    }

    # Get the environment.
    $flags       = $params['args'].reduce |$flags, $value| { "${flags} ${value}" }
    $cflags      = "CFLAGS=${flags}"
    $cxxflags    = "CXXFLAGS=${flags}"
    $environment = $environment + [$cflags, $cxxflags] + $params['environment']

    # Reduce the args array to a space separated list of arguments.
    $args = $args.reduce |$args, $value| { "${args} ${value}" }

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
        # Copy extracted archive to local arch directory, then build and install.
        # NOTE: Done as an exec to suppress to boat-loads of output that 'file' generates for recursive copies.
        exec { "mirror_${arch}_${name}":
          command     => "mkdir -p ${prefix}/${arch}/src/${name}/${src_dir} ;
                          cp -r ${prefix}/src/${name}/ ${prefix}/${arch}/src/",
          cwd         => "${prefix}/${arch}/src",
          environment => $environment,
          path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                            '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
          refreshonly => true,
          require     => [ Class['installer::prerequisites'], Class['dev_tools'], ],
        } ~>
        exec { "install_${arch}_${name}":
          command     => "${prebuild} ;
                          install_from_source '${prefix}/${arch}' '${method}' '${args}' ;
                          ${postbuild} ;",
          creates     => "${create}",
          cwd         => "${prefix}/${arch}/src/${name}/${src_dir}",
          environment => $environment,
          path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                            '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
          timeout     => 0,
          refreshonly => true,
          require     => [ Class['installer::prerequisites'], Class['dev_tools'], ],
        }
      }
    }
  }
}
