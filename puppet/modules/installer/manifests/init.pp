
class installer::prerequisites (Hash $archs) {

  # Make the generic directories.
  file { [ '/nubots',
           '/nubots/toolchain',
           '/nubots/toolchain/bin',
           '/nubots/toolchain/include',
           '/nubots/toolchain/src' ]:
    ensure => directory,
  }

  # Make the architecture specific directories.
  $archs.each |String $arch, Hash $params| {
    file { [ "/nubots/toolchain/${arch}",
             "/nubots/toolchain/${arch}/bin",
             "/nubots/toolchain/${arch}/include",
             "/nubots/toolchain/${arch}/lib",
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
  $args = undef,
  $src_dir = '.',
  $method,
  $prefix = '/nubots/toolchain',
  $prebuild = 'echo', # Colon is a noop
  $postbuild = 'echo', # Colon is a noop
  $strip_components = 1,
  $extension = 'UNKNOWN',
  Hash $archs = $archs,
) {

  $archs.each |String $arch, Hash $params| {
    # Ensure we have a valid "creates" variable.
    if $creates == '' {
      $create = "${prefix}/${arch}/lib/lib${name}.a"
    }

    else {
      $create = "${prefix}/${arch}/${creates}"
    }

    # Get the environment.
    $compiler_flags  = $params['flags'].reduce |$compiler_flags, $value| { "${compiler_flags} ${value}" }
    $compiler_params = $params['params'].reduce |$compiler_params, $value| { "${compiler_params} ${value}" }
    $flags           = "${compiler_flags} ${compiler_params} -fPIC -I${prefix}/${arch}/include -Os"
    $libzlib         = "ZLIB_LIBPATH=${prefix}/${arch}/lib"
    $inczlib         = "ZLIB_INCLUDE=${prefix}/${arch}/include"
    $libbzip         = "BZIP2_LIBPATH=${prefix}/${arch}/lib"
    $incbzip         = "BZIP2_INCLUDE=${prefix}/${arch}/include"

    # "Intelligently" merge environment variables.
    if has_key($params['environment'], 'CFLAGS') {
      $cflags = "CFLAGS=${flags} ${params['environment']['CFLAGS']}"
      $env0 = delete($params['environment'], 'CFLAGS')
    }

    else {
      $cflags = "CFLAGS=${flags}"
      $env0 = $params['environment']
    }

    if has_key($params['environment'], 'CXXFLAGS') {
      $cxxflags = "CXXFLAGS=${flags} ${params['environment']['CXXFLAGS']}"
      $env1 = delete($env0, 'CXXFLAGS')
    }

    else {
      $cxxflags = "CXXFLAGS=${flags}"
      $env1 = $env0
    }

    if has_key($params['environment'], 'COMMON_OPT') {
      $common_opt = "COMMON_OPT=${flags} ${params['environment']['COMMON_OPT']}"
      $env2 = delete($env1, 'COMMON_OPT')
    }

    else {
      $common_opt = "COMMON_OPT=${flags}"
      $env2 = $env1
    }

    if has_key($params['environment'], 'FCOMMON_OPT') {
      $fcommon_opt = "FCOMMON_OPT=${flags} ${params['environment']['FCOMMON_OPT']}"
      $env3 = delete($env2, 'FCOMMON_OPT')
    }

    else {
      $fcommon_opt = "FCOMMON_OPT=${flags}"
      $env3 = $env2
    }

    if has_key($params['environment'], 'LDFLAGS') {
      $linkflags  = "-L${prefix}/${arch}/lib -L${prefix}/lib ${params['environment']['LDFLAGS']}"
      $env4 = delete($env3, 'LDFLAGS')
    }

    else {
      $linkflags = "-L${prefix}/${arch}/lib -L${prefix}/lib"
      $env4 = $env3
    }

    $ldflags = "LDFLAGS=${linkflags}"

    if has_key($params['environment'], 'CCASFLAGS') {
      $ccasflags = "CCASFLAGS=${params['environment']['CCASFLAGS']}"
      $env5 = delete($env4, 'CCASFLAGS')
    }

    else {
      $ccasflags = "CCASFLAGS=-f elf"
      $env5 = $env4
    }

    if has_key($params['environment'], 'PKG_CONFIG_PATH') {
      $pkgconfig = "PKG_CONFIG_PATH=${params['environment']['PKG_CONFIG_PATH']}:${prefix}/${arch}/lib/pkgconfig:${prefix}/${arch}/share/pkgconfig"
      $env6 = delete($env5, 'PKG_CONFIG_PATH')
    }

    else {
      $pkgconfig = "PKG_CONFIG_PATH=${prefix}/${arch}/lib/pkgconfig"
      $env6 = $env5
    }

    if has_key($params['environment'], 'CMAKE_PREFIX_PATH') {
      $cmake_prefix = "CMAKE_PREFIX_PATH=${params['environment']['CMAKE_PREFIX_PATH']}"
      $env7 = delete($env6, 'CMAKE_PREFIX_PATH')
    }

    else {
      $cmake_prefix = "CMAKE_PREFIX_PATH=${prefix}/${arch}"
      $env7 = $env6
    }

    if has_key($params['environment'], 'LT_SYS_LIBRARY_PATH') {
      $ltsyslibpath = "LT_SYS_LIBRARY_PATH=${params['environment']['LT_SYS_LIBRARY_PATH']}"
      $env8 = delete($env7, 'LT_SYS_LIBRARY_PATH')
    }

    else {
      $ltsyslibpath = "LT_SYS_LIBRARY_PATH=${prefix}/${arch}/lib:${prefix}/lib"
      $env8 = $env7
    }

    if has_key($params['environment'], 'LD_LIBRARY_PATH') {
      $ldlibrarypath = "LD_LIBRARY_PATH=${params['environment']['LD_LIBRARY_PATH']}"
      $env9 = delete($env8, 'LD_LIBRARY_PATH')
    }

    else {
      $ldlibrarypath = "LD_LIBRARY_PATH=${prefix}/${arch}/lib:${prefix}/lib"
      $env9 = $env8
    }

    if has_key($params['environment'], 'CCAS') {
      $ccas = "CCAS=${params['environment']['CCAS']}"
      $env10 = delete($env9, 'CCAS')
    }

    else {
      $ccas = "CCAS=as"
      $env10 = $env9
    }

    if has_key($params['environment'], 'ACLOCAL_PATH') {
      $aclocalpath = "ACLOCAL_PATH=${params['environment']['ACLOCAL_PATH']}"
      $env11 = delete($env10, 'ACLOCAL_PATH')
    }

    else {
      $aclocalpath = "ACLOCAL_PATH=${prefix}/${arch}/share/aclocal:${prefix}/share/aclocal"
      $env11 = $env10
    }

    $environment = [$cflags, $cxxflags, $common_opt, $fcommon_opt,
                    $ldflags, $ccas, $ccasflags, $pkgconfig, $cmake_prefix,
                    $ltsyslibpath, $ldlibrarypath,
                    $libzlib, $inczlib, $libbzip, $incbzip, $aclocalpath, ] + join_keys_to_values($env11, "=")

    # Reduce the args array to a space separated list of arguments.
    if $args {
      $arg      = regsubst($args["${arch}"], 'ZLIB_PATH', "${prefix}/${arch}", 'G')
      $arg1     = regsubst($arg, 'PREFIX', "${prefix}/${arch}", 'G')
      $arg2     = regsubst($arg1, 'PROTOC_PATH', "${prefix}/bin/protoc", 'G')
      $args_str = $arg2.reduce |$args_str, $value| { "${args_str} ${value}" }
    }

    $prebuild_cmd = regsubst($prebuild, 'PREFIX', "${prefix}/${arch}", 'G')
    $postbuild_cmd = regsubst($postbuild, 'PREFIX', "${prefix}/${arch}", 'G')

    case $extension {
      'h', 'hpp': {
        # Download and put in include
        $basename = basename($url)

        wget::fetch { "${arch}_${name}":
          destination => "${prefix}/${arch}/include/${basename}",
          source      => $url,
          unless      => "test -e ${prefix}/${arch}/include/${basename}",
        }
      }
      default: {
        # Sync extracted archive to local arch directory, then build and install.
        # NOTE: Done as an exec to suppress to boat-loads of output that 'file' generates for recursive copies.
        if $method != 'custom' {
          exec { "mirror_${arch}_${name}":
            command     => "rsync -a ${prefix}/src/${name} .",
            cwd         => "${prefix}/${arch}/src",
            environment => $environment,
            path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                              '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
            require     => [ Archive["${name}"], ],
          }
        }

        if $method == 'autotools' {
          exec { "autotools_${arch}_${name}":
            creates     => "${create}",
            # Sometimes, autogen.sh will automatically run configure
            command     => "${prebuild_cmd} &&
                            if [ -e \"autogen.sh\" ]; then NOCONFIGURE=1 ./autogen.sh; fi &&
                            ./configure ${args_str} --prefix=\"${prefix}/${arch}\" &&
                            make -j\$(nproc) &&
                            make install &&
                            ${postbuild_cmd}",
            cwd         => "${prefix}/${arch}/src/${name}/${src_dir}",
            environment => $environment,
            path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                              '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
            timeout     => 0,
            provider    => 'shell',
            require     => [ Exec["mirror_${arch}_${name}"], ],
          }
        }

        if $method == 'cmake' {
          exec { "cmake_${arch}_${name}":
            creates     => "${create}",
            command     => "${prebuild_cmd} &&
                            if [ -d \"build\" ]; then rm -rf build; fi &&
                            mkdir build ; cd build &&
                            cmake .. ${args_str} -DCMAKE_BUILD_TYPE=\"Release\" -DCMAKE_C_FLAGS_RELEASE=\"${flags}\" -DCMAKE_CXX_FLAGS_RELEASE=\"${flags}\" -DCMAKE_INSTALL_PREFIX:PATH=\"${prefix}/${arch}\" &&
                            make -j\$(nproc) &&
                            make install &&
                            ${postbuild_cmd}",
            cwd         => "${prefix}/${arch}/src/${name}/${src_dir}",
            environment => $environment,
            path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                              '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
            timeout     => 0,
            provider    => 'shell',
            require     => [ Exec["mirror_${arch}_${name}"], ],
          }
        }

        if $method == 'boost' {
          exec { "boost_${arch}_${name}":
            creates     => "${create}",
            command     => "${prebuild_cmd} &&
                            ./bootstrap.sh --prefix=\"${prefix}/${arch}\" --with-python=python3 &&
                            ./bjam include=\"${prefix}/${arch}/include\" library-path=\"${prefix}/${arch}/lib\" ${args_str} -j\$(nproc) -q -a \\
                                  cflags=\"${flags}\" cxxflags=\"${flags}\" linkflags=\"${linkflags}\" &&
                            ./bjam install &&
                            ${postbuild_cmd}",
            cwd         => "${prefix}/${arch}/src/${name}/${src_dir}",
            environment => $environment,
            path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                              '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
            timeout     => 0,
            provider    => 'shell',
            require     => [ Exec["mirror_${arch}_${name}"], ],
          }
        }

        if $method == 'make' {
          exec { "make_${arch}_${name}":
            creates     => "${create}",
            command     => "${prebuild_cmd} &&
                            make ${args_str} -j\$(nproc) &&
                            make ${args_str} PREFIX=\"${prefix}/${arch}\" install &&
                            ${postbuild_cmd}",
            cwd         => "${prefix}/${arch}/src/${name}/${src_dir}",
            environment => $environment,
            path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                              '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
            timeout     => 0,
            provider    => 'shell',
            require     => [ Exec["mirror_${arch}_${name}"], ],
          }
        }

        if $method == 'python' {
          exec { "python_${arch}_${name}":
            creates     => "${create}",
            command     => "${prebuild_cmd} &&
                            python3 setup.py build  &&
                            python3 setup.py install --prefix=\"${prefix}/${arch}\" ${args_str} &&
                            ${postbuild_cmd}",
            cwd         => "${prefix}/${arch}/src/${name}/${src_dir}",
            environment => $environment,
            path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                              '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
            timeout     => 0,
            provider    => 'shell',
            require     => [ Exec["mirror_${arch}_${name}"], ],
          }
        }

        if $method == 'custom' {
          exec { "custom_${arch}_${name}":
            creates     => "${create}",
            command     => "mkdir -p ${name}/${src_dir} &&
                            cd ${name}/${src_dir} &&
                            ${prebuild_cmd} &&
                            ${postbuild_cmd}",
            cwd         => "${prefix}/${arch}/src",
            environment => $environment,
            path        =>  [ "${prefix}/${arch}/bin", "${prefix}/bin",
                              '/usr/local/bin', '/usr/local/sbin/', '/usr/bin/', '/usr/sbin/', '/bin/', '/sbin/' ],
            timeout     => 0,
            provider    => 'shell',
          }
        }
      }
    }
  }
}
